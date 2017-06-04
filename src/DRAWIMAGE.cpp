#include "../include/DRAWIMAGE.hpp"


class LaplacianBlending {
private:
  Mat_<Vec3f> left;
  Mat_<Vec3f> right;
  Mat_<float> blendMask;

  vector<Mat_<Vec3f> > leftLapPyr,rightLapPyr,resultLapPyr;
  Mat leftSmallestLevel, rightSmallestLevel, resultSmallestLevel;
  vector<Mat_<Vec3f> > maskGaussianPyramid; //masks are 3-channels for easier multiplication with RGB

  int levels;


  void buildPyramids() {
    buildLaplacianPyramid(left,leftLapPyr,leftSmallestLevel);
    buildLaplacianPyramid(right,rightLapPyr,rightSmallestLevel);
    buildGaussianPyramid();
  }

  void buildGaussianPyramid() {
    assert(leftLapPyr.size()>0);

    maskGaussianPyramid.clear();
    Mat currentImg;
    cvtColor(blendMask, currentImg, CV_GRAY2BGR);
    maskGaussianPyramid.push_back(currentImg); //highest level

    currentImg = blendMask;
    for (int l=1; l<levels+1; l++) {
      Mat _down;
      if (leftLapPyr.size() > l) {
        pyrDown(currentImg, _down, leftLapPyr[l].size());
      } else {
        pyrDown(currentImg, _down, leftSmallestLevel.size()); //smallest level
      }

      Mat down;
      cvtColor(_down, down, CV_GRAY2BGR);
      maskGaussianPyramid.push_back(down);
      currentImg = _down;
    }
  }

  void buildLaplacianPyramid(const Mat& img, vector<Mat_<Vec3f> >& lapPyr, Mat& smallestLevel) {
    lapPyr.clear();
    Mat currentImg = img;
    for (int l=0; l<levels; l++) {
      Mat down,up;
      pyrDown(currentImg, down);
      pyrUp(down, up, currentImg.size());
      Mat lap = currentImg - up;
      lapPyr.push_back(lap);
      currentImg = down;
    }
    currentImg.copyTo(smallestLevel);
  }

  Mat_<Vec3f> reconstructImgFromLapPyramid() {
    Mat currentImg = resultSmallestLevel;
    for (int l=levels-1; l>=0; l--) {
      Mat up;

      pyrUp(currentImg, up, resultLapPyr[l].size());
      currentImg = up + resultLapPyr[l];
    }
    return currentImg;
  }

  void blendLapPyrs() {
    resultSmallestLevel = leftSmallestLevel.mul(maskGaussianPyramid.back()) +
    rightSmallestLevel.mul(Scalar(1.0,1.0,1.0) - maskGaussianPyramid.back());
    for (int l=0; l<levels; l++) {
      Mat A = leftLapPyr[l].mul(maskGaussianPyramid[l]);
      Mat antiMask = Scalar(1.0,1.0,1.0) - maskGaussianPyramid[l];
      Mat B = rightLapPyr[l].mul(antiMask);
      Mat_<Vec3f> blendedLevel = A + B;

      resultLapPyr.push_back(blendedLevel);
    }
  }

public:
  LaplacianBlending(const Mat_<Vec3f>& _left, const Mat_<Vec3f>& _right, const Mat_<float>& _blendMask, int _levels):
  left(_left),right(_right),blendMask(_blendMask),levels(_levels)
{
  assert(_left.size() == _right.size());
  assert(_left.size() == _blendMask.size());
  buildPyramids();
  blendLapPyrs();
};

Mat_<Vec3f> blend() {
  return reconstructImgFromLapPyramid();
}
};

Mat_<Vec3f> LaplacianBlend(const Mat_<Vec3f>& l, const Mat_<Vec3f>& r, const Mat_<float>& m) {
  LaplacianBlending lb(l,r,m,4);
  return lb.blend();
}



DRAWIMAGE::~DRAWIMAGE()
{

}
DRAWIMAGE::DRAWIMAGE()
{

  REFRESH();
}
void DRAWIMAGE::REFRESH()
{
  CANVAS = Mat();
  FRAME = Mat();
  LAST = Mat();
  RESULT = Mat();

  THISXY = Point(0,0);
  LASTXY = Point(0,0);

  minY = 10000;
  maxY = 0;
}


Mat DRAWIMAGE::DRAW(Mat& thisFrame, int X, int Y, double SCALE, bool SHOWIMAGE = false)
{
  if(CANVAS.empty())
  {
    CANVAS = Mat::zeros(thisFrame.rows, thisFrame.cols*8, CV_8UC3);
  }

  if(thisFrame.channels()<3)
  {
    cvtColor(thisFrame,thisFrame, CV_GRAY2BGR);
  }

  if(X<0&&X+thisFrame.cols>CANVAS.cols)
  {
    if(X<0)
      X = 0;
    else
      X = CANVAS.cols-thisFrame.cols;

  }
  else
  {

    X = X*(0.5/SCALE);
    Y = Y*(0.5/SCALE);

    // THISXY = FILTER(Point(int(X),int(Y)),THISXY, 0.0);
    //
    double FILTER = 0.7;
    THISXY.x = X;
    THISXY.y = Y*FILTER + THISXY.y*(1-FILTER);

    X = THISXY.x;
    Y = THISXY.y;


    if(Y>=0)
    {
      ROI.x = 0;
      ROI.y = 0;
      ROI.width = thisFrame.cols;
      ROI.height = thisFrame.rows-Y;

      ROI2.x = X;
      ROI2.y = Y;
      ROI2.width = thisFrame.cols;
      ROI2.height = thisFrame.rows-Y;

      THISXY = Point(X,Y);
    }
    else
    {
      ROI.x = 0;
      ROI.y = -Y;
      ROI.width = thisFrame.cols;
      ROI.height = thisFrame.rows+Y;

      ROI2.x = X;
      ROI2.y = 0;
      ROI2.width = thisFrame.cols;
      ROI2.height = thisFrame.rows+Y;

      THISXY = Point(X,0);
    }

    pasteFrame = thisFrame(ROI).clone();
    mask = Mat::ones(pasteFrame.size(),CV_8U)*255;

    pasteFrame.copyTo(CANVAS(ROI2));

    for(int pointx=BLENDXY.x; pointx<THISXY.x; pointx++)
    {
      int pointy = ( ((THISXY.y - BLENDXY.y)*1.0/(THISXY.x - BLENDXY.x))*(pointx - THISXY.x) ) + THISXY.y;

      Point dist = Point(pointx, BLENDXY.y-pointy);
      offset.push_back(dist);
    }

    // if(BLENDXY.x-10>0)
    // {
    //   Rect r(BLENDXY.x-10, 0, 20, thisFrame.rows);
    //   CANVAS(r).convertTo(l,CV_32FC3,1.0/255.0);
    //
    //   Mat_<float> m(l.rows,l.cols,0.0);
    //   m(Range::all(),Range(0,m.cols/2)) = 1.0;
    //
    //   blend = LaplacianBlend(l, l, m);
    //   blend.convertTo(blend,CV_32F,255.0);
    //   blend.copyTo(CANVAS(r));
    //   //rectangle(CANVAS, r, Scalar(0,255,0), 1);
    //   //imwrite("mask.jpg",CANVAS(r));
    //
    //   m.release();
    // }

    // Mat second = CANVAS(r);
    // addWeighted( first, 0.5, second, 0.5, 0.0, output);
    //
    // output.copyTo(CANVAS(r));

    if(SHOWIMAGE)
    {
      SHOW("PC","RESULT",CANVAS);
      // SHOW("PC","OUTPUT",output);

    }

    LASTXY.x = THISXY.x+pasteFrame.cols;
    LASTXY.y = THISXY.y;

    BLENDXY = THISXY;

    if(LASTXY.y>maxY)// upper boundary
    {
      maxY = LASTXY.y;
    }
    if(LASTXY.y+ROI2.height<minY) // lower boundary
    {
      minY = LASTXY.y+ROI2.height;
    }
  }

  return CANVAS;

}

bool DRAWIMAGE::SAVEPANO(string path)
{
  Rect R(0,maxY,LASTXY.x,minY-maxY);
  if(!CANVAS.empty())
    imwrite(path.c_str(),CANVAS(R));
  }

Point DRAWIMAGE::FILTER(Point VAL, Point FILVAL, double FILTER)
{
  FILVAL.x = VAL.x;
  FILVAL.y = VAL.y*FILTER + FILVAL.y*(1-FILTER);
  return FILVAL;
}

Mat DRAWIMAGE::ALIGNPANO(Mat half, string outPath)
{
  // cout<<"OFFSET Size: "<<offset.size()<<endl;
  for(int i = 0; i<offset.size(); i++)
  {
    // cout<<"OFFSET : "<<offset[i]<<endl;

    int top = 0, bottom = 0;
    Rect inROI, outROI;
    inROI = Rect(offset[i].x, 0, 1, CANVAS.rows);
    outROI.x = 0;
    outROI.width = 1;

    if(offset[i].y>=0)
    {
      top = 0;
      bottom = offset[i].y;

      outROI.y = offset[i].y;
      outROI.height = CANVAS.rows;
    }
    else
    {
      top = -offset[i].y;
      bottom = 0;

      outROI.y = 0;
      outROI.height = CANVAS.rows;
    }

    // cout<<inROI<<endl<<outROI<<endl;
    Mat temp = CANVAS(inROI);
    // cout<<temp.size()<<endl;
    copyMakeBorder( temp, temp, top, bottom, 0, 0, BORDER_CONSTANT, Scalar::all(0) );
    // imshow("temp", temp);
    // waitKey(1);
    // cout<<temp.size()<<endl;
    //sif(offset[i].y<0)
     temp  = temp(outROI);
    // cout<<temp.size()<<endl;

    temp.copyTo(CANVAS(inROI));
  }

  Rect R(0,maxY,LASTXY.x,minY-maxY);
  if(!CANVAS.empty())
  {
    Mat finalPano = CANVAS(R).clone();
    hconcat(half(Rect(0,maxY,half.cols, minY-maxY)), finalPano, finalPano);
    resize(finalPano, finalPano, Size(), 0.75, 0.75);
    resize(finalPano, finalPano, Size(), 1.25, 1.25);

    // sharpen image using "unsharp mask" algorithm
    Mat blurred; double sigma = 1, threshold = 5, amount = 1;
    GaussianBlur(finalPano, blurred, Size(), sigma, sigma);
    Mat lowContrastMask = abs(finalPano - blurred) < threshold;
    Mat sharpened = finalPano*(1+amount) + blurred*(-amount);
    finalPano.copyTo(sharpened, lowContrastMask);

    imwrite(outPath.c_str(), sharpened);
    destroyAllWindows();
    namedWindow("Panorama complete!");
    moveWindow("Panorama complete!", 50,50);
    Mat endIm = imread("data/end.jpg");
    resize(endIm, endIm, Size(600*endIm.cols*1.0/endIm.rows, 600));
    putText(endIm, outPath, Point(22.0*endIm.cols/697.0, 410.0*endIm.rows/556.0),   FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255,255,255), 1);
    imshow("Panorama complete!", endIm);
    waitKey(0);

    cout<<" \n\nPanorama saved in \""<<outPath.c_str()<<"\""<<endl;
  }
  return Mat();

}
