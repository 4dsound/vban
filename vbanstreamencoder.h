#include "vban.h"

#include <functional>
#include <string>
#include <cassert>

namespace vban
{

	template <typename SenderType>
	class VBANStreamEncoder
	{
	public:
		explicit VBANStreamEncoder(SenderType& sender) : mSender(sender) { }
		virtual ~VBANStreamEncoder() = default;

		void start();
		void stop();

		template <typename T>
		void process(const T& input, int channelCount, int sampleCount);
		void setSampleRateFormat(int format) { mSampleRateFormat = format; }
		void setChannelCount(int value) { mChannelCount = value; }
		void setStreamName(const std::string& name) { mStreamName = name; }

		const std::string& getStreamName() const { return mStreamName; }
		int isRunning() const { return mIsRunning; }

	private:
		// Settings
		int mSampleRateFormat = 0; // Index to VBanSRList, sample rates supported by VBAN
		int mAudioBufferSize = 0; //
		int mChannelCount = 2; // Number of channels of audio being sent

		std::vector<char> mVbanBuffer = { }; // Data containing the full VBAN packet including the header
		VBanHeader *mPacketHeader = nullptr; // Pointer to packet header within mVbanBuffer
		int mPacketSize = 0;
		int mPacketChannelSize = 0; // Size in bytes of one channel of audio in the VBAN packet

		// State
		std::string mStreamName = "vbanstream";
		int mPacketWritePos = VBAN_HEADER_SIZE; // Write position in the mVbanBuffer of incoming audio data.
		int mPacketCounter = 0; // Number of packets sent
		bool mIsRunning = false;

		SenderType& mSender;
	};


	template <typename SenderType> template <typename T>
	void VBANStreamEncoder<SenderType>::process(const T& input, int channelCount, int sampleCount)
	{
		if (!mIsRunning)
			return;

		for (auto i = 0; i < sampleCount; ++i)
		{
			for (auto channel = 0; channel < mChannelCount; ++channel)
			{
				float sample = input[channel][i];
				short value = static_cast<short>(sample * 32768.0f);

				// convert short to two bytes
				char byte_1 = value;
				char byte_2 = value >> 8;
				mVbanBuffer[mPacketWritePos] = byte_1;
				mVbanBuffer[mPacketWritePos + 1] = byte_2;

				mPacketWritePos += 2;
			}
			if (mPacketWritePos >= mPacketSize)
			{
				assert(mPacketWritePos == mPacketSize);
				mPacketHeader->nuFrame = mPacketCounter;
				mSender.sendPacket(mVbanBuffer.data(), mPacketWritePos);
				mPacketWritePos = VBAN_HEADER_SIZE;
				mPacketCounter++;
			}
		}
	}


	template <typename SenderType>
	void VBANStreamEncoder<SenderType>::start()
	{
		if (mIsRunning)
			return;

		// Resize packet channel size to fit max data size
		mPacketChannelSize = int(VBAN_SAMPLES_MAX_NB / mChannelCount) * 2;

		// compute the buffer size of all channels together
		mAudioBufferSize = mPacketChannelSize * mChannelCount;

		// set packet size
		mPacketSize = mAudioBufferSize + VBAN_HEADER_SIZE;

		// resize the packet data to have the correct size
		mVbanBuffer.resize(mPacketSize);

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
		mPacketHeader->format_nbs = (mPacketChannelSize / 2) - 1;

		mIsRunning = true;
	}


	template <typename SenderType>
	void VBANStreamEncoder<SenderType>::stop()
	{
		mIsRunning = false;
	}

}