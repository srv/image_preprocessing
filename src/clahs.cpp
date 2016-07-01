/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <image_preprocessing/clahs.h>
#include <math.h>

Mat Clahs::makeLUT(int min, int max, int nr_bins) {

}


// To speed up histogram clipping, the input image [Min,Max] is scaled down to
// [0,NrBins-1].  This function calculates the LUT.
void MakeLUT(const int& Min, 
			 const int& Max, 
			 const int& NrBins, 
			 	   Mat& LUT)
{
	int BinSize = 1 + floor((Max-Min)/NrBins);
	int interval = Max-Min;
	Mat i = linspace(Min+1,Max+1, interval);
	for (int j = 0; j < interval; ++j)
	{
		LUT[j] = min(1 + floor((j-Min)/BinSize),NrBins);
	}
}


// This function performs clipping of the histogram and redistribution of the 
// clipped bin elements.  Histogram is clipped and excess elements are counted
// Then excess elements are equally reditributed over the whole histogram
// (providing the bin count is smaller than the cliplimit)

void ClipHistogram(const Mat& Histogram, 
				   const int& NrGreyLevels, 
				   const int& ClipLimit,
				   		 Mat& NewHistogram)
{
	% number of excess pixels created by clipping
	double NrExcess = sum(max(Histogram-ClipLimit,0));

	// # of elements to be redist'ed to each bin  
	int BinIncr	= floor(NrExcess/NrGreyLevels);

	// max bin value where redist. will be above Climit
	int Upper = ClipLimit - BinIncr; 
	
	// clip the histogram to ClipLimit
	NewHistogram = min(Histogram, ClipLimit);

	// add partial BinIncr pixels to bins up to ClipLimit
	Mat ii = NewHistogram > Upper;
	Mat H(ii.rows,1,CV_64FC1);
	for (int i = 0; i < len(ii); ++i)
	{
		H.row(i) = NewHistogram.row(ii.row(i));
	}
	double NrExcess = NrExcess - sum(add(ClipLimit, - H));

	for (int j = 0; j < len(ii); ++j)
	{
		NewHistogram.row(ii.row(j)) = ClipLimit;
	}

	// add BinIncr to all other bins
	Mat jj = NewHistogram <= Upper;

	double NrExcess = NrExcess - len(jj)*BinIncr;

	for (int j = 0; j < len(jj); ++j)
	{
		NewHistogram.row(jj.row(j)) = NewHistogram.row(jj.row(j)) + BinIncr;
	}

	///////////	
	// evenly redistribute remaining excess pixels

	while (NrExcess>0)
	{
	    int h = 1;
	    while ((h < NrGreyLevels) && (NrExcess > 0)) 
	    {
	        // choose step to distribute the most excess evenly in one pass 
	        int StepSize = ceil(NrGreyLevels/NrExcess);
	        int i = h;
	        while ((i<(NrGreyLevels+1)) && (NrExcess >0))
	        { 
	            if (NewHistogram.row(i) < ClipLimit)
	            {
	                NewHistogram.row(i) =  NewHistogram.row(i) + 1;
	                NrExcess = NrExcess - 1;
	            }
	            i = i + StepSize; // step along the histogram
	        }
	        h = h + 1; // avoid concentrating pixels in bin 1
	    }
	}

}

void ClipHistogramSimple(const Mat& Histogram,
				   		 const int& ClipLimit,
				   		 	   Mat& NewHistogram)
{
	// This function performs clipping of the histogram
	// any bin with a value above the cliplimit is assigned the value of ClipLimit

	NewHistogram = min(Histogram,ClipLimit);
}

void MapHistogram(const Mat& Histogram, 
				  const int& Min, 
				  const int& Max, 
				  const int& NrGreyLevels, 
				  const int& NrofPixels, 
				  const int& heq_type,
				  const int& heq_alpha)
{
	// This function calculates the equalized lookup table (mapping)
	// by cumulating the input histogram
	// Note: Lookup table is rescaled in the range [Min..Max].

	int HEQ_NONE        = 0;
	int HEQ_UNIFORM     = 1; 
	int HEQ_EXPONENTIAL = 2;
	int HEQ_RAYLEIGH    = 3;

	double Scale = (Max-Min)/NrofPixels;

	switch (heq_type)
	{
		case HEQ_UNIFORM:  //accummulate histogram uniformly
		    double Sum = sum(Histogram);
		    // scale it from min to max
		    Histogram = min(Min + Sum*Scale, Max); //limit range to Max

		case HEQ_EXPONENTIAL: //accumulate histogram exponentially 
		    double Sum = sum(Histogram);
		    double vmax = 1.0 - exp(-heq_alpha);
		    double temp = -1/heq_alpha*log(1-(vmax*Sum/NrofPixels));
		    Histogram = min(temp*Max,Max); //limit range to Max

		case HEQ_RAYLEIGH: //accumulate histogram using rayleigh
		    double Sum = sum(Histogram);
		    double hconst = 2*heq_alpha*heq_alpha;
		    double vmax = 1 - exp((-1/hconst));
		    double temp = sqrt(-hconst*log(1-vmax*(double(Sum)/NrofPixels)));
		    Histogram = min(temp*Max,Max); //limit range to Max

		default: //just do UNIFORM if heq_type has a wacky value
		    double Sum = sum(Histogram);
		    Histogram = min(Min + Sum*Scale,Max); //limit range to Max
	}    
}

void Interpolate()
{

}


// Extra functions
Mat linspace(const double& startP,
			  const double& Endp,
			  const int& interval)
{
    double spacing = interval-1;
    Mat y(spacing,1,CV_64FC1);
    for (int i = 0; i < y.rows; ++i)
    {
        y.at<double>(i) = startP + i*(Endp - startP)/spacing;
    }
    return y;
}