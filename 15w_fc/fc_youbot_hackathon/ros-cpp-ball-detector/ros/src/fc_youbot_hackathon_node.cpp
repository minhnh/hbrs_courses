#include <fc_youbot_hackathon/ball_detector.h>

using namespace cv;

int main( int argc, char * argv[] ) {
    Mat img = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    if ( !img.data ) {
        std::cout << "Invalid file name!"  << std::endl;
        return 1;
    }

    /* Convert to HSV format */
    Mat hsv;
    cvtColor( img, hsv, CV_BGR2HSV );

    /* Two masks are used to handle wrap around cases
     * Note: Ranges of HSV in OpenCV are
     *      - H: 0-180
     *      - S: 0-255
     *      - V: 0-255 */
    //Size size = hsv.size();
    Mat mask( hsv.size(), CV_8UC1 );
    Mat mask_lower( hsv.size(), CV_8UC1 );
    inRange(hsv, Scalar( 0.96*180, 0.23*256, 0.17*256 ),
                    Scalar( 1.00*180, 1.00*256, 0.99*256 ), mask);
    inRange(hsv, Scalar( 0.00*180, 0.23*256, 0.17*256 ),
                    Scalar( 0.02*180, 1.00*256, 0.99*256 ), mask_lower);

    bitwise_or(mask, mask_lower, mask);
    //std::cout << "Got to line 31"  << std::endl;

    /* Morphological operations to eliminate noises */
    Mat se21 = getStructuringElement(MORPH_RECT, Size( 2*10 + 1, 2*10+1 ), Point( 10, 10 ));
    Mat se11 = getStructuringElement(MORPH_RECT, Size( 2*4 + 1, 2*4+1 ),  Point( 4, 4 ));
    morphologyEx(mask, mask, MORPH_CLOSE, se11); // See completed example for cvClose definition
    morphologyEx(mask, mask, MORPH_OPEN, se21);  // See completed example for cvOpen  definition

    namedWindow( "Mask Image", CV_WINDOW_AUTOSIZE );
    imshow( "Mask Image", mask );
    waitKey(0);
    destroyWindow( "Mask Image" );
    return 0;
}


