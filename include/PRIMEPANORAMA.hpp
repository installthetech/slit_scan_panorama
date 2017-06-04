#include "LIBRARIES.hpp"
#include "UTILITY.hpp"

class PRIMEPANO
{
  //Variables declaration
  public:

    // Mat containers
    Mat FRAME, LASTFRAME;

    // Feature matching tools
    Ptr<FeatureDetector> detector;
    Ptr<DescriptorExtractor> extractor;
    Ptr<DescriptorMatcher> matcher;
    std::vector<KeyPoint> THISPOINTS;
    std::vector<KeyPoint> LASTPOINTS;
    Mat THISDESC, LASTDESC;
    std::vector<DMatch> matches;
    std::vector<DMatch> good_matches;

    //Feature matching params
    int MAXFEAT, MAXDIS, EDGETHRESH, FIRSTLEVEL;
    int PATCHSIZE, FASTTHRESH;
    double SCALEFACTOR;

    //Translation params
    double XDIS, YDIS; //relative translation
    double XABS, YABS; //absolute translation

    int XLAST, YLAST; // last edited segment's info

    //Image warping params
    Mat warped, warpmask;
    double FOCAL;
    Rect warpROI;
    Point OFFSET;

    //Seam finding and blending params
    vector<Mat> images_warped;
    vector<Mat> masks;
    vector<Point> corners;
    vector<cv::Size> sizeVector;
    vector<UMat> masks_warped_f;

    Ptr<ExposureCompensator> compensator;
    Ptr<SeamFinder> seam_finder;
    Ptr<cv::detail::Blender> blender;
    cv::detail::MultiBandBlender* mb;
    float blend_strength;
    Size dst_sz;
    float blend_width;

  // Functions declaration
  public:
    PRIMEPANO();
    ~PRIMEPANO();
    void REFRESH(); // resetting function
    int CALCFEAT(Mat&, bool); //current, showimage -> numFeatures
    int MATCHFEAT(Mat& , Mat& , bool); //current, last, showimage -> numFeatures
    Point GETRELATION(); // return translation (X,Y)
    bool WARPIMAGE(Mat&, double, double,  bool); // current, FOV, scaleReduced, showimage
    bool SEAMBLENDER(double, bool); // seamScale, showImage

};
