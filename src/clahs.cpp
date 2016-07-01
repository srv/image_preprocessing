/// Copyright 2014 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <image_preprocessing/clahs.h>
#include <algorithm>

Clahs::Clahs() {}

Mat Clahs::compute(Mat Image, int NrY, int NrX) {

  // Initializations
  const int Min                   = 0;
  const int Max                   = 255;
  const int NrBins                = 256;
  const int NrGreyLevels          = 256;
  const int YRes                  = Image.rows;
  const int XRes                  = Image.cols;
  const int heq_type              = 3;
  const double alpha              = 0.4;
  const int MAX_REG_X             = 128;
  const int MAX_REG_Y             = 128;
  const int HEQ_CLIP_SIMPLE       = 1;
  const int HEQ_CLIP_REDISTRIBUTE = 2;

  int ClipLimit                   = 5;
  int clip_type                   = 2;

  // Check if image is uint8
  Mat I;
  if (Image.type() != CV_8U) {
    ROS_WARN("[ImagePreprocessing]: Input image type is not CV_8U. It will be converted to grayscale CV_8U.");
    Image.convertTo(I,CV_8U);
  } else {
    Image.copyTo(I);
  }

  // Sanity checks
  if ( NrX > MAX_REG_X )
    ROS_ERROR("[ImagePreprocessing]: NrX > MAX_REG_X.");
  if (NrY > MAX_REG_Y)
    ROS_ERROR("[ImagePreprocessing]: NrY > MAX_REG_Y.");
  if ( (XRes/NrX) % 2 != 0 )
    ROS_ERROR_STREAM("[ImagePreprocessing]: XRes/NrX = " << XRes/NrX << ", noninteger value.");
  if ( (YRes/NrY) % 2 != 0 )
    ROS_ERROR_STREAM("[ImagePreprocessing]: YRes/NrY = " << YRes/NrY << ", noninteger value.");
  if ( Max > NrGreyLevels )
    ROS_ERROR_STREAM("[ImagePreprocessing]: Max image value > number of greylevels.");
  if ( NrX<2 || NrY<2 )
    ROS_ERROR_STREAM("[ImagePreprocessing]: NrX<2 or NrY<2, number of subregions must be>=4.");

  // Setup/initialization
  Mat MapArray = Mat::zeros(NrX*NrY*NrBins,1,CV_64FC1);

  // Determine properties of each subregion
  int XSize = XRes/NrX;
  int YSize = YRes/NrY;
  int NrPixels = XSize*YSize;

  // calculate actual clip limit
  if (ClipLimit > 1) {
    ClipLimit = (ClipLimit*NrPixels/NrBins);
    if (ClipLimit < 1)
      ClipLimit = 1;
    else
      ClipLimit = ClipLimit;
  } else {
    // large value, do not clip (AHE)
    ClipLimit = std::numeric_limits<int>::max();
    clip_type = 0;
  }

  // calculate greylevel mappings for each subregion
  // calculate and process histograms for each subregion
  for (int Y=0; Y<(NrY-1); Y++) {
    for (int X=0; X<(NrX-1); X++) {
      int Xtmp = X*XSize+1;
      int Ytmp = Y*YSize+1;
      Mat SubRegion = I(Range(Ytmp, Ytmp+YSize-1), Range(Xtmp, Xtmp+XSize-1));

      Mat Hist;
      float range[] = { 0, 256 } ;
      const float* hist_range = { range };
      if (NrBins >= NrGreyLevels) {
        calcHist( &SubRegion, 1, 0, Mat(), Hist, 1, &NrBins, &hist_range, true, false );
      } else {
        ROS_ERROR("[ImagePreprocessing]: NrBins must be greater or equal to NrGreyLevels");
      }

      Hist.convertTo(Hist,CV_64FC1);

      // clip histogram, simple or redistribute depending on input parameters
      if (clip_type == HEQ_CLIP_REDISTRIBUTE) {
        Hist = ClipHistogram(Hist,NrBins,ClipLimit);
      } else if (clip_type == HEQ_CLIP_SIMPLE) {
        Hist  = ClipHistogramSimple(Hist,ClipLimit);
      }

      // create histogram mapping (uniform,exponential,or rayleigh)
      Hist = MapHistogram(Hist, Min, Max, NrBins, NrPixels, heq_type, alpha);

      // write working histogram into appropriate part of MapArray
      Hist.copyTo( MapArray( Rect( (NrBins*(Y*NrX+X))+1, 0, Hist.cols, Hist.rows) ) );
    }
  }

  // interpolate greylevel mappings to get CLAHE image
  // make lookup table for mapping of grey values
  Mat LUT = MakeLUT(NrGreyLevels, Min, Max, NrBins);
  Mat CLAHSImage = Mat::zeros(YRes,XRes,CV_8U);
  int lenY = 1;

  for (int Y=0; Y<NrY; Y++) {
    int SubY, YU, YB;
    if (Y == 0) {       // special case top row
      SubY = floor(YSize/2);
      YU = 0;
      YB = 0;
    } else if (Y == NrY) { //special case bottom row
      SubY = floor(YSize/2);
      YU = NrY-1;
      YB = YU;
    } else {
      SubY = YSize;
      YU = Y-1;
      YB = YU+1;
    }
    int lenX = 1;
    for (int X=0; X<NrX; X++) {
      int SubX, XL, XR;
      if (X==0) {       // special case Left column
        SubX = floor(XSize/2);
        XL = 0;
        XR = 0;
      } else if (X == NrX) { //special case right column
        SubX = floor(XSize/2);
        XL = NrX-1;
        XR = XL;
      } else {
        SubX = XSize;
        XL = X-1;
        XR = XL+1;
      }

      // retrieve the appropriate histogram mappings from MapArray
      Mat LU = MapArray( Range((NrBins*(YU*NrX +XL))+1, ((NrBins*(YU*NrX +XL)))+NrBins), Range(0, 0) );
      Mat RU = MapArray( Range((NrBins*(YU*NrX +XR))+1, ((NrBins*(YU*NrX +XR)))+NrBins), Range(0, 0) );
      Mat LB = MapArray( Range((NrBins*(YB*NrX +XL))+1, ((NrBins*(YB*NrX +XL)))+NrBins), Range(0, 0) );
      Mat RB = MapArray( Range((NrBins*(YB*NrX +XR))+1, ((NrBins*(YB*NrX +XR)))+NrBins), Range(0, 0) );

      // interpolate the appropriate subregion
      Mat SubRegion = I(Range(lenY, lenY+SubY-1), Range(lenX, lenX+SubX-1));
      Mat InterpD = Interpolate(SubRegion,LU,RU,LB,RB,SubX,SubY,LUT);
      InterpD.copyTo( CLAHSImage( Rect( lenY, lenX, InterpD.cols, InterpD.rows) ) );

      lenX = lenX+SubX;
    }
    lenY = lenY+SubY;
  }

  return CLAHSImage;
}

Mat Clahs::MakeLUT(const int& NrGreyLevels,
                   const int& Min,
                   const int& Max,
                   const int& NrBins) {
  Mat LUT = Mat::zeros(NrGreyLevels,1,CV_64FC1);
  int BinSize = 1 + floor((Max-Min)/NrBins);
  int interval = Max-Min;
  Mat i = linspace(Min+1,Max+1, interval);
  for (int j = 0; j < interval; ++j) {
    LUT.at<double>(j,0) = std::min(1.0 + floor((j-Min)/BinSize), (double)NrBins);
  }
  return LUT;
}

Mat Clahs::ClipHistogram(const Mat& Histogram,
                         const int& NrGreyLevels,
                         const int& ClipLimit) {
  Mat NewHistogram;

  // number of excess pixels created by clipping
  Mat max_hist;
  max(Histogram-ClipLimit,0,max_hist);
  double NrExcess = sum(max_hist)[0];

  // # of elements to be redist'ed to each bin
  int BinIncr = floor(NrExcess/NrGreyLevels);

  // max bin value where redist. will be above Climit
  int Upper = ClipLimit - BinIncr;

  // clip the histogram to ClipLimit
  NewHistogram = min(Histogram, ClipLimit);

  // add partial BinIncr pixels to bins up to ClipLimit
  Mat ii = NewHistogram > Upper;
  Mat H(ii.rows,1,CV_64FC1);
  for (int i = 0; i < ii.rows; ++i) {
    H.at<double>(i,0) = NewHistogram.at<double>(ii.at<double>(i,0));
  }
  NrExcess -= sum(ClipLimit-H)[0];

  for (int j = 0; j < ii.rows; ++j) {
    NewHistogram.at<double>(ii.at<double>(j,0)) = ClipLimit;
  }

  // add BinIncr to all other bins
  Mat jj = NewHistogram <= Upper;

  NrExcess -= jj.rows*BinIncr;

  for (int j = 0; j < jj.rows; ++j) {
    NewHistogram.at<double>(jj.at<double>(j,0)) += BinIncr;
  }

  ///////////
  // evenly redistribute remaining excess pixels

  while (NrExcess>0) {
    int h = 1;
    while ((h < NrGreyLevels) && (NrExcess > 0)) {
      // choose step to distribute the most excess evenly in one pass
      int StepSize = ceil(NrGreyLevels/NrExcess);
      int i = h;
      while ((i<(NrGreyLevels+1)) && (NrExcess >0)) {
        if (NewHistogram.at<double>(i,0) < ClipLimit) {
          NewHistogram.row(i) =  NewHistogram.row(i) + 1;
          NrExcess = NrExcess - 1;
        }
        i = i + StepSize; // step along the histogram
      }
      h = h + 1; // avoid concentrating pixels in bin 1
    }
  }
  return NewHistogram;
}

Mat Clahs::ClipHistogramSimple(const Mat& Histogram,
                               const int& ClipLimit) {
  // This function performs clipping of the histogram
  // any bin with a value above the cliplimit is assigned the value of ClipLimit

  return min(Histogram,ClipLimit);
}

Mat Clahs::MapHistogram(const Mat& Histogram,
                        const int& Min,
                        const int& Max,
                        const int& NrGreyLevels,
                        const int& NrofPixels,
                        const int& heq_type,
                        const int& heq_alpha) {
  // This function calculates the equalized lookup table (mapping)
  // by cumulating the input histogram
  // Note: Lookup table is rescaled in the range [Min..Max].
  Mat output;
  Histogram.copyTo(output);

  const int HEQ_NONE        = 0;
  const int HEQ_UNIFORM     = 1;
  const int HEQ_EXPONENTIAL = 2;
  const int HEQ_RAYLEIGH    = 3;

  double Scale = (Max-Min)/NrofPixels;

  switch (heq_type) {
    case HEQ_UNIFORM:
    {  //accummulate histogram uniformly
      Mat Sum = cumsum(Histogram);
      // scale it from min to max
      output = min(Min + Sum*Scale, Max); //limit range to Max
      break;
    }
    case HEQ_EXPONENTIAL:
    { //accumulate histogram exponentially
      Mat Sum = cumsum(Histogram);
      double vmax = 1.0 - exp(-heq_alpha);
      Mat dst;
      log(1-(vmax*Sum/NrofPixels), dst);
      Mat temp = (-1/heq_alpha)*dst;
      output = min(temp*Max,Max); //limit range to Max
      break;
    }
    case HEQ_RAYLEIGH:
    { //accumulate histogram using rayleigh
      Mat Sum = cumsum(Histogram);
      double hconst = 2*heq_alpha*heq_alpha;
      double vmax = 1 - exp((-1/hconst));
      Mat dst;
      log(1-vmax*(Sum/NrofPixels), dst);
      Mat temp;
      sqrt(-hconst*dst, temp);
      output = min(temp*Max,Max); //limit range to Max
      break;
    }
    default:
    { //just do UNIFORM if heq_type has a wacky value
      Mat Sum = cumsum(Histogram);
      output = min(Min + Sum*Scale,Max); //limit range to Max
      break;
    }
  }

  return output;
}


Mat Clahs::Interpolate(const Mat& SubRegion,
                       const Mat& MapLU,
                       const Mat& MapRU,
                       const Mat& MapLB,
                       const Mat& MapRB,
                       const int& XSize,
                       const int& YSize,
                       const Mat& LUT) {
  int Num = XSize * YSize; //Normalization factor
  Mat BinValues = buildbyIndices(LUT, SubRegion);

  Mat XInvCoef;
  int dec1 = XSize;
  Mat row1 = Mat::zeros(1,XSize,CV_8U);
  for (uint i=0; i<XSize; i++) {
    row1.at<uint>(i,0) = dec1;
    dec1--;
  }
  for (uint j=0; j<YSize; j++) {
    XInvCoef.push_back(row1);
  }

  uint dec2 = YSize;
  Mat YInvCoef = Mat::zeros(YSize,XSize,CV_8U);
  for (uint i=0; i<YSize; i++) {
    for (uint j=0; j<XSize; j++) {
      YInvCoef.at<uint>(i,j) = dec2;
    }
    dec2--;
  }

  Mat XCoef;
  int inc1 = 0;
  Mat row2 = Mat::zeros(1,XSize,CV_8U);
  for (uint i=0; i<XSize; i++) {
    row2.at<uint>(i,0) = inc1;
    inc1++;
  }
  for (uint j=0; j<YSize; j++) {
    XCoef.push_back(row2);
  }

  uint inc2 = 0;
  Mat YCoef = Mat::zeros(YSize,XSize,CV_8U);
  for (uint i=0; i<YSize; i++) {
    for (uint j=0; j<XSize; j++) {
      YCoef.at<uint>(i,j) = inc2;
    }
    inc2++;
  }

  Mat MapLU_red = buildbyIndices(MapLU, BinValues);
  Mat MapRU_red = buildbyIndices(MapRU, BinValues);
  Mat MapLB_red = buildbyIndices(MapLB, BinValues);
  Mat MapRB_red = buildbyIndices(MapRB, BinValues);
  Mat InterpRegion = Mat::zeros(MapLU_red.rows,MapLU_red.cols,CV_64FC1);
  for (uint i=0; i<MapLU_red.rows; i++) {
    for (uint j=0; j<MapLU_red.cols; j++) {
      InterpRegion.at<double>(i,j) = YInvCoef.at<double>(i,j) * ( XInvCoef.at<double>(i,j)*MapLU_red.at<double>(i,j) + XCoef.at<double>(i,j)*MapRU_red.at<double>(i,j) ) +
                                     YCoef.at<double>(i,j) * ( XInvCoef.at<double>(i,j)*MapLB_red.at<double>(i,j) + XCoef.at<double>(i,j)*MapRB_red.at<double>(i,j) );
    }
  }

  return InterpRegion;
}

Mat Clahs::buildbyIndices(const Mat& vector, const Mat& indices) {
  Mat output = Mat::zeros(indices.rows,indices.cols,CV_8U);
  for (int i=0; i<indices.rows; i++) {
    for (int j=0; j<indices.cols; j++) {
      uint idx = indices.at<uint>(i,j) + 1;
      output.at<uint>(i,j) = vector.at<uint>(idx,0);
    }
  }
  return output;
}

Mat Clahs::linspace(const double& startP,
                    const double& Endp,
                    const int& interval) {
  double spacing = interval-1;
  Mat y(spacing,1,CV_64FC1);
  for (int i = 0; i < y.rows; ++i) {
    y.at<double>(i) = startP + i*(Endp - startP)/spacing;
  }
  return y;
}

Mat Clahs::cumsum(const Mat& in) {
  Mat output = Mat::zeros(in.rows,1,in.type());
  for (uint i=0; i<in.rows; i++) {
    if (i>0) {
      output.at<double>(i,0) = output.at<double>(i-1,0) + in.at<double>(i,0);
    } else {
      output.at<double>(i,0) = in.at<double>(i,0);
    }
  }
  return output;
}