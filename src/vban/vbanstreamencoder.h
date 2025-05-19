#include "vban.h"
#include "dirtyflag.h"

#include <functional>
#include <string>
#include <cassert>
#include <mutex>
#include <vector>

namespace vban
{

	/**
	 * Helper class to encode a multichannel audio signal into a VBAN packet stream that can be send through an external protocol.
	 * @tparam SenderType The type of the sender object that is invoked by the encoder to send the VBAN packets.
	 * 	The SenderType has to implement the sendPacket() method with the following signature:
	 * 	SenderType::sendPacket(const std::vector<char>& data);
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
		void setSampleRateFormat(int format);

		/**
		 * Sets the buffer size of the calling audio processing system.
		 * The vban packet size will adapt to this buffer size in order to achieve the lowest possible latency.
		 * @param bufferSize in samples
		 */
		void setBufferSize(int bufferSize);

		/**
		 * Sets the bit depth of the audio data, or the number of bits per sample.
		 * At the moment only 16 and 32 bit audio is supported. Other values result in a runtime error.
		 * @param bitDepth The desired number of bits per sample
		 */
		void setBitDepth(int bitDepth);

		/**
		 * Sets the number of audio channels encoded in the stream.
		 * @param value
		 */
		void setChannelCount(int value);

		/**
		 * Sets the name of the stream as encoded in the packets.
		 * @param name Has to be equal or smaller than 16 characters.
		 */
		void setStreamName(const std::string& name);

		/**
		 * Activates or deactivates the vban encoding.
		 * @param value True on activate, false on deactivate
		 */
		void setActive(bool value);

		/**
		 * @return Whether the encoder is running and sending VBAN packets.
		 */
		int isActive() const { return mIsActive.load(); }

		/**
		 * @return Number of channels being sent by the stream.
		 */
		int getChannelCount() const { return mChannelCount.load(); }

	private:
		/**
		 * Updates the internal state from the current settings
		 */
		void update();

		// Settings
		std::atomic<int> mSampleRateFormat = { 0 }; // Index to VBanSRList, sample rates supported by VBAN
		std::atomic<int> mChannelCount = { 2 }; // Number of channels of audio being sent
		std::atomic<int> mBufferSize = { 256 }; // Buffer size of the audio processing
		std::atomic<int> mBitDepth = { 16 }; // Bit depth of the vban data
		std::atomic<bool> mIsActive = { false };
		DirtyFlag mIsDirty;

		// State
		std::string mStreamName = "vbanstream";
		std::mutex mStreamNameLock;
		int mPacketWritePos = VBAN_HEADER_SIZE; // Write position in the mVbanBuffer of incoming audio data.
		int mPacketCounter = 0; // Number of packets sent
		int mCurrentChannelCount = 0; // Current channelcount
		int mBytesPerSample = 2; // Determined from bit depth setting

		// VBAN packet
		std::vector<char> mVbanBuffer; // Data containing the full VBAN packet including the header
		VBanHeader *mPacketHeader = nullptr; // Pointer to packet header within mVbanBuffer

		SenderType& mSender;
	};


	template <typename SenderType> template <typename T>
	void VBANStreamEncoder<SenderType>::process(const T& input, int channelCount, int sampleCount)
	{
		if (!mIsActive)
			return;

		if (mIsDirty.check())
			update();

		for (auto i = 0; i < sampleCount; ++i)
		{
			for (auto channel = 0; channel < mCurrentChannelCount; ++channel)
			{
				float sample = input[channel][i];
				if (sample < -1.f)
					sample = -1.f;
				if (sample > 1.f)
					sample = 1.f;

				if (mBytesPerSample == 4)
				{
					auto value = static_cast<int32_t>(sample * std::numeric_limits<int32_t>::max());

					// convert 32 bit int to four bytes
					mVbanBuffer[mPacketWritePos] = value;
					mVbanBuffer[mPacketWritePos + 1] = value >> 8;
					mVbanBuffer[mPacketWritePos + 2] = value >> 16;
					mVbanBuffer[mPacketWritePos + 3] = value >> 24;
					mPacketWritePos += 4;
				}
				else {
					auto value = static_cast<int16_t>(sample * std::numeric_limits<int16_t>::max());

					// convert 16 bit int to two bytes
					mVbanBuffer[mPacketWritePos] = value;
					mVbanBuffer[mPacketWritePos + 1] = value >> 8;
					mPacketWritePos += 2;
				}
			}
			if (mPacketWritePos >= mVbanBuffer.size())
			{
				assert(mPacketWritePos == mVbanBuffer.size());
				mPacketHeader->nuFrame = mPacketCounter;
				mSender.sendPacket(mVbanBuffer);
				mPacketWritePos = VBAN_HEADER_SIZE;
				mPacketCounter++;
			}
		}
	}


	template <typename SenderType>
	void VBANStreamEncoder<SenderType>::setSampleRateFormat(int format)
	{
		assert(format < VBAN_SR_MAXNUMBER);
		mSampleRateFormat.store(format);
		mIsDirty.set();
	}


	template<typename SenderType>
	void VBANStreamEncoder<SenderType>::setBufferSize(int bufferSize)
	{
		mBufferSize.store(bufferSize);
		mIsDirty.set();
	}


	template<typename SenderType>
	void VBANStreamEncoder<SenderType>::setBitDepth(int bitrate)
	{
		assert(bitrate == 16 || bitrate == 32);
		mBitDepth.store(bitrate);
		mIsDirty.set();
	}


	template <typename SenderType>
	void VBANStreamEncoder<SenderType>::setChannelCount(int value)
	{
		assert(value <= VBAN_CHANNELS_MAX_NB);
		mChannelCount.store(value);
		mIsDirty.set();
	}


	template <typename SenderType>
	void VBANStreamEncoder<SenderType>::setStreamName(const std::string& name)
	{
		assert(name.size() <= 16);
		std::lock_guard<std::mutex> lock(mStreamNameLock);
		mStreamName = name;
		mIsDirty.set();
	}


	template <typename SenderType>
	void VBANStreamEncoder<SenderType>::setActive(bool value)
	{
		mIsActive.store(value);
		mIsDirty.set();
	}


	template <typename SenderType>
	void VBANStreamEncoder<SenderType>::update()
	{
		mCurrentChannelCount = mChannelCount.load();

		mBytesPerSample = 2;
		if (mBitDepth.load() == 32)
			mBytesPerSample = 4;

		// Determine the packet size
		// Ideally the packet holds one single buffer of the calling DSP system
		int samplesPerPacket = mBufferSize.load();
		if (samplesPerPacket > VBAN_SAMPLES_MAX_NB)
			samplesPerPacket = VBAN_SAMPLES_MAX_NB;
		int samplesSize = samplesPerPacket * mBytesPerSample * mCurrentChannelCount;
		if (samplesSize > VBAN_DATA_MAX_SIZE)
		{
			samplesPerPacket = (VBAN_DATA_MAX_SIZE / mBytesPerSample) / mCurrentChannelCount;
			samplesSize = samplesPerPacket * mBytesPerSample * mCurrentChannelCount;
		}

		auto packetSize = samplesSize + VBAN_HEADER_SIZE;

		// resize the packet data to have the correct size
		mVbanBuffer.resize(packetSize);

		// Reset packet counter and buffer write position
		mPacketCounter = 0;
		mPacketWritePos = VBAN_HEADER_SIZE;

		// initialize VBAN header
		mPacketHeader = (struct VBanHeader*)(&mVbanBuffer[0]);
		mPacketHeader->vban       = *(int32_t*)("VBAN");
		mPacketHeader->format_nbc = mCurrentChannelCount - 1;
		mPacketHeader->format_SR  = mSampleRateFormat.load();
		if (mBytesPerSample == 4)
			mPacketHeader->format_bit = VBAN_BITFMT_32_INT;
		else
			mPacketHeader->format_bit = VBAN_BITFMT_16_INT;
		{
			std::lock_guard<std::mutex> lock(mStreamNameLock);
			strncpy(mPacketHeader->streamname, mStreamName.c_str(), VBAN_STREAM_NAME_SIZE - 1);
		}
		mPacketHeader->nuFrame    = mPacketCounter;
		mPacketHeader->format_nbs = samplesPerPacket - 1;
	}

}
