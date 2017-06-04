#include "LIBRARIES.hpp"
#include "UTILITY.hpp"

class DRAWIMAGE
{
  public:
    Mat CANVAS; // CANVAS to draw images
  private:
    Mat FRAME, LAST; // Mat containers
    Point THISXY, LASTXY, BLENDXY; // THIS start and LAST end
    int minY, maxY; // for cropping of final panorama
    Mat RESULT; // final panorama
    Rect ROI; // image rect to be cropped
    Rect ROI2; // canvas rect to be filled

    Mat pasteFrame, mask; // image cropped
    // blending params
    Mat_<Vec3f> blend;
    Mat_<Vec3f> l;

    vector<Point> offset;

  public:
    DRAWIMAGE();
    ~DRAWIMAGE();
    void REFRESH();
    Mat BLEND(Mat&, Mat&);
    Mat DRAW(Mat&, int, int, double, bool); // currentImage, X, Y, showImage -> lastfilled (X,Y)
    bool SAVEPANO(string);
    Point FILTER(Point, Point, double);
    Mat ALIGNPANO(Mat, string);

};
