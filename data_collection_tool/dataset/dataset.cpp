#include "opencv2/opencv.hpp"
#include <chrono>
using namespace cv;
using namespace std;


int i = 1756, j = 0, k = 1999 , l = 2999;
int main(int, char**)
{
    
    // Create a VideoCapture object and open the input file
   // If the input is the web camera, pass 0 instead of the video file name

    VideoCapture cap("C://dataset5.mp4");
    /*VideoCapture cap2("C://dataset2.mp4");
    VideoCapture cap3("C://dataset3.mp4");*/


    // Check if camera opened successfully
    if (!cap.isOpened()) {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    while (1) {

        Mat frame, frame2, frame3;
        Mat showing, showing2, showing3;

        // Capture frame-by-frame

        cap >> frame;
        /*cap2 >> frame2;
        cap3 >> frame3;*/

        // If the frame is empty, break immediately
        if (frame.empty() && frame2.empty() && frame3.empty())
            break;


        // Display the resulting frame
        transpose(frame, showing);
        flip(showing, showing,  1);
        /*transpose(showing, frame);
        transpose(frame, frame);
        flip(frame, frame, 1);
        transpose(frame, frame);*/

        //flip(frame, frame, 1);

/*        transpose(frame2, showing2);
        flip(showing2, showing2, 1);

        transpose(frame3, showing3);
        flip(showing3, showing3, 1);*/

        //frame_rotated = rotate(frame, -90);
        imshow("Frame", showing);
        /*imshow("Frame2", showing2);
        imshow("Frame3", showing3);*/
        double fps = cap.get(CAP_PROP_FPS);

        if (true)
        {
            i++;
            if (i < 2057)
            {
                string name = "notobj_" + to_string(i) + ".jpg";
                imwrite("C://Users/USER/Desktop/labelimg/data/" + name, showing);
            }

            
            /*
            if (i % 7 == 0)
            {
                j++;
                string name = to_string(j) + ".jpg";
                imwrite("C://Users/USER/Desktop/labelimg/data/" + name, showing);

                k++;
                string name2 = to_string(k) + ".jpg";
                imwrite("C://Users/USER/Desktop/labelimg/data/" + name2, showing2);

                l++;
                string name3 = to_string(l) + ".jpg";
                imwrite("C://Users/USER/Desktop/labelimg/data/" + name3, showing3);
            }
            */

        }

        
        // Press  ESC on keyboard to exit
        char c = (char)waitKey(25);
        if (c == 27)
            break;
    }

    // When everything done, release the video capture object
    cap.release();

    // Closes all the frames
    destroyAllWindows();


    return 0;
}