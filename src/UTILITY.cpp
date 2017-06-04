#include "../include/UTILITY.hpp"

double euclideanDist(Point2f& p, Point2f& q) {
  Point2f diff = p - q;
  return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}

void PRINT(string TYPE, string msg)
{
  if (TYPE.compare("ANDROID") == 0)// returns 0 when same
  {
    // __android_log_print(ANDROID_LOG_INFO, "IM C CODE", "%s", msg.c_str());
  }

  if (TYPE.compare("PC") == 0)// returns 0 when same
  {
    printf("%s",msg.c_str());
  }
}

void PRINT(string TYPE, string msg, int val)
{
  if (TYPE.compare("ANDROID") == 0)// returns 0 when same
  {
    // __android_log_print(ANDROID_LOG_INFO, "IM C CODE", "%s : %d", msg.c_str(), val);
  }

  if (TYPE.compare("PC") == 0)// returns 0 when same
  {
    printf("%s : %d",msg.c_str(), val);
  }
}

void PRINT(string TYPE, string msg, float val)
{
  if (TYPE.compare("ANDROID") == 0)// returns 0 when same
  {
    // __android_log_print(ANDROID_LOG_INFO, "IM C CODE", "%s : %f", msg.c_str(), val);
  }

  if (TYPE.compare("PC") == 0)// returns 0 when same
  {
    printf("%s : %f",msg.c_str(), val);
  }
}

void SHOW(string TYPE, string msg, Mat img)
{
  if (TYPE.compare("ANDROID") == 0)// returns 0 when same
  {
    imwrite(msg.c_str(), img);
  }

  if (TYPE.compare("PC") == 0)// returns 0 when same
  {
    imshow(msg.c_str(), img);
    waitKey(1);
  }
}
