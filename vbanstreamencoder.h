#include "vban.h"

#include <functional>
#include <string>
#include <cassert>

namespace vban
{

	/**
	 * Helper class to encode a multichannel audio signal into a VBAN packet stream that can be send through an external protocol.
	 * @tparam SenderType The type of the sender object that is invoked by the encoder to send the VBAN packets.
	 * 	The SenderType has to implement the sendPacket() method with the following signature:
	 * 	SenderType::sendPacket(char* data, int size);
	 * 	With data being a pointer to the VBAN packet data and size the size of the packet.
	 */
	template <typename SenderType>
	class VBANStreamEncoder
	{
	public:
		/**
		 * Constructor
		 * @param sender This object's sendPacket() method will be called by the encoder to send VBAN packets.
		 */
		explicit VBANStreamEncoder(SenderType& sender) : mSender(sender) { }

		// Default destructor
		virtual ~VBANStreamEncoder() = default;

		/**
		 * Activates the encoding process and start sending packets.
		 */
		void start();

		/**
		 * Deacitvates the encoding process.
		 */
		void stop();

		/**
		 * Call this method to process incoming sample from the multichannel audio signal.
		 * @tparam T Type for the multichannel audio data. Implements a subscript operator that returns data for a single channel.
		 * 	Data for a single channel also needs to implement a subscript operator that returns samples as floating point values.
		 * 	Examples: std::vector<std::vector<float>> or double**
		 * @param input Multichannel audio data.
		 * @param dataChannelCount Number of channels in input. This has to be greater than or equal to the number of channels in the stream, set by setChannelCount().
		 * @param dataSampleCount Number of samples in input.
		 */
		template <typename T>
		void process(const T& input, int dataChannelCount, int dataSampleCount);

		/**
		 * Sets the sample rate of the stream to one of the supported VBAN sample rate formats.
		 * @param format Index to one of the sample rate formats specified in VBanSRList
		 */
		void setSampleRateFormat(int format)
		{
			assert(format < VBAN_SR_MAXNUMBER);
			mSampleRateFormat = format;
			reset();
		}

		/**
		 * Sets the number of audio channels encoded in the stream.
		 * @param value
		 */
		void setChannelCount(int value)
		{
			assert(value <= VBAN_CHANNELS_MAX_NB);
			mChannelCount = value;
			reset();
		}

		/**
		 * Sets the name of the stream as encoded in the packets.
		 * @param name Has to be equal or smaller than 16 characters.
		 */
		void setStreamName(const std::string& name)
		{
			assert(mStreamName.size() <= 16);
			mStreamName = name;
			reset();
		}

		/**
		 * @return The name of the VBAN stream.
		 */
		const std::string& getStreamName() const { return mStreamName; }

		/**
		 * @return Whether the encoder is running and sending VBAN packets.
		 */
		int isActive() const { return mIsActive; }

	private:
		/**
		 * Restarts the encoder if it is active.
		 */
		void reset();

		// Settings
		int mSampleRateFormat = 0; // Index to VBanSRList, sample rates supported by VBAN
		int mChannelCount = 2; // Number of channels of audio being sent

		std::vector<char> mVbanBuffer = { }; // Data containing the full VBAN packet including the header
		VBanHeader *mPacketHeader = nullptr; // Pointer to packet header within mVbanBuffer

		// State
		std::string mStreamName = "vbanstream";
		int mPacketWritePos = VBAN_HEADER_SIZE; // Write position in the mVbanBuffer of incoming audio data.
		int mPacketCounter = 0; // Number of packets sent
		bool mIsActive = false;

		SenderType& mSender;
	};


	template <typename SenderType> template <typename T>
	void VBANStreamEncoder<SenderType>::process(const T& input, int channelCount, int sampleCount)
	{
		if (!mIsActive)
			return;

		for (auto i = 0; i < sampleCount; ++i)
		{
			for (auto channel = 0; channel < mChannelCount; ++channel)
			{
				float sample = input[channel][i];
				auto value = static_cast<short>(sample * 32768.0f);

				// convert short to two bytes
				char byte_1 = value;
				char byte_2 = value >> 8;
				mVbanBuffer[mPacketWritePos] = byte_1;
				mVbanBuffer[mPacketWritePos + 1] = byte_2;

				mPacketWritePos += 2;
			}
			if (mPacketWritePos >= mVbanBuffer.size())
			{
				assert(mPacketWritePos == mVbanBuffer.size());
				mPacketHeader->nuFrame = mPacketCounter;
				mSender.sendPacket(mVbanBuffer.data(), mVbanBuffer.size());
				mPacketWritePos = VBAN_HEADER_SIZE;
				mPacketCounter++;
			}
		}
	}


	template <typename SenderType>
	void VBANStreamEncoder<SenderType>::start()
	{
		if (mIsActive)
			return;

		// Determine size of a single channel
		auto channelSize = int(VBAN_SAMPLES_MAX_NB / mChannelCount) * 2;

		// set packet size
		auto packetSize = channelSize * mChannelCount + VBAN_HEADER_SIZE;

		// resize the packet data to have the correct size
		mVbanBuffer.resize(packetSize);

		// Reset packet counter and buffer write position
		mPacketCounter = 0;
		mPacketWritePos = VBAN_HEADER_SIZE;

		// initialize VBAN header
		mPacketHeader = (struct VBanHeader*)(&mVbanBuffer[0]);
		mPacketHeader->vban       = *(int32_t*)("VBAN");
		mPacketHeader->format_nbc = mChannelCount - 1;
		mPacketHeader->format_SR  = mSampleRateFormat;
		mPacketHeader->format_bit = VBAN_BITFMT_16_INT;
		strncpy(mPacketHeader->streamname, mStreamName.c_str(), VBAN_STREAM_NAME_SIZE - 1);
		mPacketHeader->nuFrame    = mPacketCounter;
		mPacketHeader->format_nbs = (channelSize / 2) - 1;

		mIsActive = true;
	}


	template <typename SenderType>
	void VBANStreamEncoder<SenderType>::stop()
	{
		mIsActive = false;
	}


	template <typename SenderType>
	void VBANStreamEncoder<SenderType>::reset()
	{
		if (mIsActive)
		{
			stop();
			start();
		}
	}

}