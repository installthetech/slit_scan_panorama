#include "../include/PRIMEPANORAMA.hpp"


bool response_comparator(const DMatch& p1, const DMatch& p2) {
  return p1.distance < p2.distance;
}

PRIMEPANO::~PRIMEPANO()
{

}
PRIMEPANO::PRIMEPANO()
{

  // detector params
  MAXFEAT = 50;
  SCALEFACTOR = 1.2f;
  EDGETHRESH = 31;//31
  FIRSTLEVEL = 0;
  PATCHSIZE = 31;//31
  FASTTHRESH = 30;
  detector = ORB::create(MAXFEAT,SCALEFACTOR, 8,EDGETHRESH,FIRSTLEVEL,2,ORB::HARRIS_SCORE, PATCHSIZE,FASTTHRESH);

  // descriptor & matcher initialization
  extractor = ORB::create();
  matcher = DescriptorMatcher::create("BruteForce-Hamming");

  // compensation of exposure
  compensator = ExposureCompensator::createDefault(ExposureCompensator::GAIN_BLOCKS);

  //Seam finders available

  //seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR_GRAD);
  //seam_finder = makePtr<detail::VoronoiSeamFinder>();
  //seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);
  //seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR_GRAD);
  seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR);

  // blender params initialization
  blend_strength = 3;
  blender = cv::detail::Blender::createDefault(cv::detail::Blender::MULTI_BAND);
  mb = static_cast<cv::detail::MultiBandBlender*>(blender.get());

}

void PRIMEPANO::REFRESH()
{
  //initializing focal length to ZERO
  FOCAL = 0;

  //translation params initialization
  XDIS = 0, YDIS = 0;
  XABS = 0, YABS = 0;
  XLAST = 0, YLAST = 0;

  // images resetted
  FRAME = Mat();
  LASTFRAME = Mat();

  //descriptor resetted
  THISDESC = Mat();
  LASTDESC = Mat();

  //features resetted
  THISPOINTS.clear();
  LASTPOINTS.clear();

  //matches resetted
  matches.clear();
  good_matches.clear();

  // seam-blending params resetted
  images_warped.clear();
  masks.clear();
  corners.clear();
  sizeVector.clear();
  masks_warped_f.clear();

}


int PRIMEPANO::CALCFEAT(Mat& thisFrame, bool SHOWIMAGE = false)
{
  if(thisFrame.channels()>1)
    cvtColor(thisFrame,thisFrame,CV_BGR2GRAY);

  if(THISPOINTS.size()>0)
  {
    LASTPOINTS = THISPOINTS;
    LASTDESC = THISDESC.clone();
  }

  detector->detect(thisFrame, THISPOINTS);
  extractor->compute(thisFrame, THISPOINTS, THISDESC);

  if(SHOWIMAGE)
  {
    Mat tempFrame = thisFrame.clone();
    drawKeypoints(tempFrame, THISPOINTS,tempFrame, Scalar(0,0,255));
    drawKeypoints(tempFrame, LASTPOINTS,tempFrame, Scalar(255,0,5));

    SHOW("PC","keypoints",tempFrame);
  }

  return THISPOINTS.size();

}


int PRIMEPANO::MATCHFEAT(Mat& thisFrame, Mat& lastFrame, bool SHOWIMAGE)
{
  if(thisFrame.channels()>1)
    cvtColor(thisFrame,thisFrame,CV_BGR2GRAY);

  if(lastFrame.channels()>1)
    cvtColor(lastFrame,lastFrame,CV_BGR2GRAY);

  if(THISPOINTS.size()==0||LASTPOINTS.size()==0)
  {
    // PRINT("PC","UnFEATURES FAILED");
    return false;
  }

  matches.clear();
  matcher->match(THISDESC, LASTDESC, matches);

  static int FEATSIZE = 10;
  if(matches.size()<FEATSIZE)
  {
    FEATSIZE = matches.size();
  }

  std::sort(matches.begin(), matches.end(), response_comparator);
  std::nth_element(matches.begin(),matches.begin()+FEATSIZE, matches.end());
  matches.erase(matches.begin()+FEATSIZE+1, matches.end());
  matches.erase(matches.begin(),matches.begin()+1);

  good_matches.clear();
  double avgDist = 0;
  for(int i = 0; i<matches.size();i++)
  {
    if(euclideanDist(LASTPOINTS[matches[i].trainIdx].pt,THISPOINTS[matches[i].queryIdx].pt)<=thisFrame.rows/15)
    {
      //cout<<i<<" : "<<matches[i].distance<<" - "<<euclideanDist(LASTPOINTS[matches[i].trainIdx].pt, THISPOINTS[matches[i].queryIdx].pt)<<endl;
      good_matches.push_back(matches[i]);
      avgDist += euclideanDist(LASTPOINTS[matches[i].trainIdx].pt,THISPOINTS[matches[i].queryIdx].pt);
    }
  }

  XDIS = 0;
  YDIS = 0;
  if(good_matches.size()==0)
  {
    return 0;
  }
  else
  {
    int counter = 0;
    for(int i = 0;i< good_matches.size();i++)
    {

      counter++;
      if(abs(LASTPOINTS[good_matches[i].trainIdx].pt.y - THISPOINTS[good_matches[i].queryIdx].pt.y)<=(avgDist*1.0/good_matches.size()))
      {
        counter++;
        XDIS += (LASTPOINTS[good_matches[i].trainIdx].pt.x - THISPOINTS[good_matches[i].queryIdx].pt.x);
        YDIS += (LASTPOINTS[good_matches[i].trainIdx].pt.y - THISPOINTS[good_matches[i].queryIdx].pt.y);
      }
    }
    if(counter>0)
    {
      XDIS *= (1.0/counter);
      YDIS *= (1.0/counter);
    }
    else
    {
      XDIS = 0;
      YDIS = 0;
      return 0;
    }
  }

  // SHOWING MATCHES ON THE IMAGES
  if(SHOWIMAGE&&good_matches.size()>0)
  {
    Mat img_matches;
    drawMatches(thisFrame, THISPOINTS, lastFrame, LASTPOINTS, good_matches, img_matches, Scalar(0,0,0), Scalar(0,255,0), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    // stringstream ss;
    // ss<<"/storage/emulated/0/DCIM/ReosCamera/match/zfet"<<XFILTER<<".jpg";
    // imwrite(ss.str(),img_matches);
    // SHOW("PC","matches", img_matches);
  }

  return good_matches.size();

}


Point PRIMEPANO::GETRELATION()
{
  XABS += XDIS;
  YABS += YDIS;

  return Point(XABS, YABS);
}



bool PRIMEPANO::WARPIMAGE(Mat& thisColorFrame, double FOV, double SCALE,  bool SHOWIMAGE = false)
{

  if(FOCAL == 0)
    FOCAL = (thisColorFrame.cols*SCALE/2.0)*(1.0/(tan((FOV*CV_PI/180.0)/2.0)));

  warped = Mat::zeros(thisColorFrame.size(), CV_8UC3);
  warpmask = Mat::zeros(thisColorFrame.size(), CV_8UC1);
  //Mat warped;
  //Mat warpmask;
  for(int i=0;i<thisColorFrame.cols;i++)
  {
    for(int j=0;j<thisColorFrame.rows;j++)
    {
      double newX = FOCAL*atan((i-thisColorFrame.cols/2)/FOCAL);
      double newY = FOCAL*(j-thisColorFrame.rows/2)/(sqrt(((i-thisColorFrame.cols/2)*(i-thisColorFrame.cols/2))+(FOCAL*FOCAL)));

      //double newX = FOCAL*tan((i-thisColorFrame.cols/2)/FOCAL);
      //double newY = FOCAL*((j-thisColorFrame.rows/2)/FOCAL)*sqrt(1+pow(tan((i-thisColorFrame.cols/2)/FOCAL),2));

      newX += thisColorFrame.cols/2;
      newY += thisColorFrame.rows/2;

      if(newX<0)
        newX = 0;
      else if(newX>=thisColorFrame.cols-1)
      newX=thisColorFrame.cols-1;

      if(newY<0)
        newY = 0;
      else if(newY>=thisColorFrame.rows-1)
      newY=thisColorFrame.rows-1;

      //cout<<i<<" "<<j<<" - "<<int(newX)<<" : "<<newY<<" ,"<<thisColorFrame.size()<<endl;
      warped.at<Vec3b>(int(newY),int(newX))[0] = int(thisColorFrame.at<Vec3b>(j,i)[0]);
      warped.at<Vec3b>(int(newY),int(newX))[1] = int(thisColorFrame.at<Vec3b>(j,i)[1]);
      warped.at<Vec3b>(int(newY),int(newX))[2] = int(thisColorFrame.at<Vec3b>(j,i)[2]);

      //if(mask.empty())
      warpmask.at<uchar>(int(newY),int(newX)) = 255;

    }
  }


  warpROI.x = FOCAL*atan((0-thisColorFrame.cols/2)/FOCAL)+thisColorFrame.cols/2;
  warpROI.width = FOCAL*atan((thisColorFrame.cols-thisColorFrame.cols/2)/FOCAL)+thisColorFrame.cols/2- warpROI.x;
  //warpROI.y = FOCAL*(0-thisColorFrame.rows/2)/(sqrt(((thisColorFrame.cols/2-thisColorFrame.cols/2)*(thisColorFrame.cols/2-thisColorFrame.cols/2))+(FOCAL*FOCAL)))+thisColorFrame.rows/2;
  warpROI.y = FOCAL*(0-thisColorFrame.rows/2)/(sqrt(((0-thisColorFrame.cols/2)*(0-thisColorFrame.cols/2))+(FOCAL*FOCAL)))+thisColorFrame.rows/2;
  warpROI.height = FOCAL*(thisColorFrame.rows-thisColorFrame.rows/2)/(sqrt(((0-thisColorFrame.cols/2)*(0-thisColorFrame.cols/2))+(FOCAL*FOCAL)))+thisColorFrame.rows/2- 2*warpROI.y;

  OFFSET.x = thisColorFrame.cols/2+(FOCAL*atan((0-thisColorFrame.cols/2)/FOCAL))-warpROI.x;
  OFFSET.y = thisColorFrame.rows/2+(FOCAL*(0-thisColorFrame.rows/2)/(sqrt(((0-thisColorFrame.cols/2)*(0-thisColorFrame.cols/2))+(FOCAL*FOCAL))))-warpROI.y;

  //circle(warped, Point(warpROI.x, warpROI.y), 4, Scalar(5, 0,250),3);

  warped = warped(warpROI);
  warpmask = warpmask(warpROI);
  //circle(warped, OFFSET, 4, Scalar(255, 255,0),3);

  resize(warped, warped, thisColorFrame.size());
  thisColorFrame = warped.clone();

  if(SHOWIMAGE)
  {
    imshow("Warped",warped);
    imshow("Mask",warpmask);
    waitKey(1);
  }
  warped.release();
  return true;
}
