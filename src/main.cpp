#include "../include/PRIMEPANORAMA.hpp"
#include "../include/DRAWIMAGE.hpp"
#include <string> // for string comparison

void overlayImage(const cv::Mat&, const cv::Mat&, cv::Mat&, cv::Point2i);

PRIMEPANO PRIME;
DRAWIMAGE DRAWIM;
Point LASTFILL = Point(0,0);
VideoCapture cap;
Mat frame, last, errorIm, warnIm;
Mat canvas, half;
Mat orig;
double FOV = 66.0;
double SCALE = 1.0;
bool START = true;
int rescaled_size = 720;
int breakCount = 0;
int main(int argc, char** argv)
{
  if(argc<3)
  {
	cout<<"Usage: ./panorama camera_index/video_path output_image_name\n";
	return 0;
  }

  string video_path = argv[1];
  string directory = "output/";
  string output_path = directory+argv[2];

  if (video_path.compare("0") == 0 || video_path.compare("1") == 0 || video_path.compare("2") == 0)
  {
  	cap.open(atoi(video_path.c_str()));
  }
  else
  {
  	cap.open(video_path);	
  }

  if(!cap.isOpened())
  {
  	cout<<"Unable to read video stream!\n";
  	return 0;
  }

  errorIm = imread("data/error.png", IMREAD_UNCHANGED);
  resize(errorIm, errorIm, Size(120.0*(errorIm.cols*1.0/errorIm.rows), 120));
  warnIm = imread("data/warn.jpg");
  
  while(cap.isOpened()&&waitKey(1)<0)
  {
    cap>>frame;
    if(!frame.empty())
    {
    	if(rescaled_size>frame.rows)
    	{
    		rescaled_size = frame.rows;
    	}

	    resize(frame, frame, Size(rescaled_size*(frame.cols*1.0/frame.rows),rescaled_size));
	    Mat tempFrame = frame.clone();
	    if(START)
	    {
	    	START = false;
	    	half = tempFrame(Rect(0, 0, tempFrame.cols*0.4, tempFrame.rows));
	    }
	    frame = frame(Rect(frame.cols*0.40,0, frame.cols*0.5, frame.rows));
	    orig = frame.clone();
	    // cout<<frame.size()<<endl;
	    resize(frame, frame, Size(), SCALE, SCALE);

	    if(last.empty())
	    {
	      PRIME.CALCFEAT(frame, false);
	      last = frame.clone();
	      continue;
	    }

	    int feat = PRIME.CALCFEAT(frame, false);
	    int match = PRIME.MATCHFEAT(frame, last, true);
	    // cout<<trans<<" : "<<LASTFILL.x <<endl;
	    //if(trans.x-(LASTFILL.x)>frame.cols*0.1||trans.x-(LASTFILL.x)==0)
	    if(match>0)
	    {
	      Point trans = PRIME.GETRELATION();
	      if(trans.x>0)
	      	canvas = DRAWIM.DRAW(orig, trans.x, trans.y, SCALE/4.0, false);
	      else
	      {
		    canvas = DRAWIM.DRAW(orig, 0, 0, SCALE/4.0, false);
		    overlayImage(tempFrame, errorIm, tempFrame, cv::Point(tempFrame.cols-errorIm.cols,tempFrame.rows/3-errorIm.rows/2));
	      }


	      resize(warnIm, warnIm, Size(tempFrame.cols,tempFrame.cols*warnIm.rows*1.0/warnIm.cols));
	      addWeighted(tempFrame(Rect(0, tempFrame.rows-warnIm.rows, warnIm.cols, warnIm.rows)), 0.5, warnIm, 0.5, 1.0, tempFrame(Rect(0, tempFrame.rows-warnIm.rows, warnIm.cols, warnIm.rows)));

	      resize(canvas, canvas, Size(tempFrame.cols, tempFrame.cols*(canvas.rows*1.0/canvas.cols)));
	      vconcat(tempFrame, canvas, tempFrame);
	      resize(tempFrame, tempFrame, Size(600.0*(tempFrame.cols*1.0/tempFrame.rows),600));

	      namedWindow("Building panorama image...");
	      moveWindow("Building panorama image...", 50,50);
	      imshow("Building panorama image...", tempFrame);
	      
	      LASTFILL = trans;
	      last = frame.clone();

	    }
	    else
	    {
	    	breakCount++;
	    	if(breakCount>3)
	    	{
	    		cout<<"Unable to get enough information to make panorama! \nSaving the current progress in panorama."<<endl;
	    		DRAWIM.ALIGNPANO(half, output_path);
	    		return 0;
	    	}
	    }
	}
	else
	{
		break;
	}

  }
  // DRAWIM.SAVEPANO("output/pano.jpg");
  DRAWIM.ALIGNPANO(half, output_path);
  return 0;
}

// http://jepsonsblog.blogspot.in/2012/10/overlay-transparent-image-in-opencv.html
void overlayImage(const cv::Mat &background, const cv::Mat &foreground, 
  cv::Mat &output, cv::Point2i location)
{
  background.copyTo(output);


  // start at the row indicated by location, or at row 0 if location.y is negative.
  for(int y = std::max(location.y , 0); y < background.rows; ++y)
  {
    int fY = y - location.y; // because of the translation

    // we are done of we have processed all rows of the foreground image.
    if(fY >= foreground.rows)
      break;

    // start at the column indicated by location, 

    // or at column 0 if location.x is negative.
    for(int x = std::max(location.x, 0); x < background.cols; ++x)
    {
      int fX = x - location.x; // because of the translation.

      // we are done with this row if the column is outside of the foreground image.
      if(fX >= foreground.cols)
        break;

      // determine the opacity of the foregrond pixel, using its fourth (alpha) channel.
      double opacity =
        ((double)foreground.data[fY * foreground.step + fX * foreground.channels() + 3])

        / 255.;


      // and now combine the background and foreground pixel, using the opacity, 

      // but only if opacity > 0.
      for(int c = 0; opacity > 0 && c < output.channels(); ++c)
      {
        unsigned char foregroundPx =
          foreground.data[fY * foreground.step + fX * foreground.channels() + c];
        unsigned char backgroundPx =
          background.data[y * background.step + x * background.channels() + c];
        output.data[y*output.step + output.channels()*x + c] =
          backgroundPx * (1.-opacity) + foregroundPx * opacity;
      }
    }
  }
}
