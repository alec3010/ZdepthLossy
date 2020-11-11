#include "zdepthcaller.hpp"

zdepth::DepthCompressor* CreateDepthCompressor()
{
    return new zdepth::DepthCompressor();
}

void DisposeDepthCompressor(zdepth::DepthCompressor* a_pObject)
{
    if(a_pObject!=NULL)
    {
        delete a_pObject;
        a_pObject=NULL;
    }
}
int GetFrameCount(zdepth::DepthCompressor* a_pCompressor)
{
	return (int)a_pCompressor->FrameCount;
}
int CSDecompress(zdepth::DepthCompressor* a_pCompressor,
				uint8_t* comp,
				int comp_len,
				uint16_t* depth_out
				)
{
	if (a_pCompressor != NULL)
	{
		
		int width;
		int height;

		std::vector<uint8_t> compressed(comp, comp + comp_len);
		//compressed.assign;
		std::vector<uint16_t> out;
		//out.assign;

		zdepth::DepthResult result = a_pCompressor->Decompress(compressed,
							  width,
							  height,
							  out
							  );
		//const int size = (int)(width * height);
		
		depth_out = out.data();
		

		if (result == zdepth::DepthResult::Success)
		{
			return width * height;
		}
		else if (result == zdepth::DepthResult::FileTruncated)
		{
			return 1;
		}
		else if (result == zdepth::DepthResult::WrongFormat)
		{
			return 2;
		}
		else if (result == zdepth::DepthResult::Corrupted)
		{
			return 3;
		}
		else if (result == zdepth::DepthResult::MissingFrame)
		{
			return 4;
		}

		
	}
	else
	{
		return 0;
	}
}

