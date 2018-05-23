#include "CMT.h"
#include "gui.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <string>
#include <algorithm>

#ifdef __GNUC__
#include <getopt.h>
#else
#include "getopt/getopt.h"
#endif

using cmt::CMT;
using cv::imread;
using cv::namedWindow;
using cv::Scalar;
using cv::VideoCapture;
using cv::waitKey;
using std::cerr;
using std::istream;
using std::ifstream;
using std::stringstream;
using std::ofstream;
using std::cout;
using std::min_element;
using std::max_element;
using std::endl;
using ::atof;

static string WIN_NAME = "CMT";
static string OUT_FILE_COL_HEADERS =
    "Frame,Timestamp (ms),Active points,"\
    "Bounding box centre X (px),Bounding box centre Y (px),"\
    "Bounding box width (px),Bounding box height (px),"\
    "Bounding box rotation (degrees),"\
    "Bounding box vertex 1 X (px),Bounding box vertex 1 Y (px),"\
    "Bounding box vertex 2 X (px),Bounding box vertex 2 Y (px),"\
    "Bounding box vertex 3 X (px),Bounding box vertex 3 Y (px),"\
    "Bounding box vertex 4 X (px),Bounding box vertex 4 Y (px)";

vector<float> getNextLineAndSplitIntoFloats(istream& str)
{
    vector<float>   result;
    string                line;
    getline(str,line);

    stringstream          lineStream(line);
    string                cell;
    while(getline(lineStream,cell,','))
    {
        result.push_back(atof(cell.c_str()));
    }
    return result;
}

int display(Mat im, CMT & cmt)
{
    //Visualize the output
    //It is ok to draw on im itself, as CMT only uses the grayscale image
    for(size_t i = 0; i < cmt.points_active.size(); i++)
    {
        circle(im, cmt.points_active[i], 2, Scalar(255,0,0));
    }

    Point2f vertices[4];
    cmt.bb_rot.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        line(im, vertices[i], vertices[(i+1)%4], Scalar(255,0,0));
    }

    imshow(WIN_NAME, im);

    return waitKey(5);
}

string write_rotated_rect(RotatedRect rect)
{
    Point2f verts[4];
    rect.points(verts);
    stringstream coords;

    coords << rect.center.x << "," << rect.center.y << ",";
    coords << rect.size.width << "," << rect.size.height << ",";
    coords << rect.angle << ",";

    for (int i = 0; i < 4; i++)
    {
        coords << verts[i].x << "," << verts[i].y;
        if (i != 3) coords << ",";
    }

    return coords.str();
}

int main(int argc, char **argv)
{
    //Create a CMT object
    CMT cmt;

    //Initialization bounding box
    Rect rect;

    //Parse args
    int loop_flag = 0;
    int ipcamera_flag = 0;
    int verbose_flag = 0;
    int bbox_flag = 0;
    int skip_frames = 0;
    int skip_msecs = 0;
    int output_flag = 0;
    string input_path;
    string output_path;

    const int detector_cmd = 1000;
    const int descriptor_cmd = 1001;
    const int bbox_cmd = 1002;
    const int no_scale_cmd = 1003;
    const int with_rotation_cmd = 1004;
    const int skip_cmd = 1005;
    const int skip_msecs_cmd = 1006;
    const int output_file_cmd = 1007;

    struct option longopts[] =
    {
        //No-argument options
        {"ipcamera", no_argument, &ipcamera_flag, 1},
        {"loop", no_argument, &loop_flag, 1},
        {"verbose", no_argument, &verbose_flag, 1},
        {"no-scale", no_argument, 0, no_scale_cmd},
        {"with-rotation", no_argument, 0, with_rotation_cmd},
        //Argument options
        {"bbox", required_argument, 0, bbox_cmd},
        {"detector", required_argument, 0, detector_cmd},
        {"descriptor", required_argument, 0, descriptor_cmd},
        {"output-file", required_argument, 0, output_file_cmd},
        {"skip", required_argument, 0, skip_cmd},
        {"skip-msecs", required_argument, 0, skip_msecs_cmd},
        {0, 0, 0, 0}
    };

    int index = 0;
    int c;
    while((c = getopt_long(argc, argv, "vi", longopts, &index)) != -1)
    {
        switch (c)
        {
            case 'i':
                ipcamera_flag = true;
                break;
            case 'v':
                verbose_flag = true;
                break;
            case bbox_cmd:
                {
                    //TODO: The following also accepts strings of the form %f,%f,%f,%fxyz...
                    string bbox_format = "%f,%f,%f,%f";
                    float x,y,w,h;
                    int ret = sscanf(optarg, bbox_format.c_str(), &x, &y, &w, &h);
                    if (ret != 4)
                    {
                        cerr << "bounding box must be given in format " << bbox_format << endl;
                        return 1;
                    }

                    bbox_flag = 1;
                    rect = Rect(x,y,w,h);
                }
                break;
            case output_file_cmd:
                output_path = optarg;
                output_flag = 1;
                break;
            case skip_cmd:
                {
                    int ret = sscanf(optarg, "%d", &skip_frames);
                    if (ret != 1)
                    {
                      skip_frames = 0;
                    }
                }
                break;
            case skip_msecs_cmd:
                {
                    int ret = sscanf(optarg, "%d", &skip_msecs);
                    if (ret != 1)
                    {
                      skip_msecs = 0;
                    }
                }
                break;
            case no_scale_cmd:
                cmt.consensus.estimate_scale = false;
                break;
            case with_rotation_cmd:
                cmt.consensus.estimate_rotation = true;
                break;
            case '?':
                return 1;
        }

    }

    // Can only skip frames or milliseconds, not both.
    if (skip_frames > 0 && skip_msecs > 0)
    {
      cerr << "You can only skip frames, or milliseconds, not both." << endl;
      return 1;
    }

    //One argument remains
    if (optind == argc - 1)
    {
        input_path = argv[optind];
    }

    else if (optind < argc - 1)
    {
        cerr << "Only one argument is allowed." << endl;
        return 1;
    }

    //Set up logging
    FILELog::ReportingLevel() = verbose_flag ? logDEBUG : logINFO;
    Output2FILE::Stream() = stdout; //Log to stdout

    //Normal mode

    //Create window
    namedWindow(WIN_NAME);

    VideoCapture cap;

    bool show_preview = true;

    // First check ipcamera_flag
    if(ipcamera_flag)
    {
        cap.open("rtspsrc location=rtsp://192.168.1.168:554/sub latency=0 ! decodebin ! videoconvert ! appsink");
    }

    //If no input was specified
    else if (input_path.length() == 0)
    {
        cap.open(0); //Open default camera device
    }

    //Else open the video specified by input_path
    else
    {
        cap.open(input_path);

        if (skip_frames > 0)
        {
          cap.set(CV_CAP_PROP_POS_FRAMES, skip_frames);
        }

        if (skip_msecs > 0)
        {
          cap.set(CV_CAP_PROP_POS_MSEC, skip_msecs);

          // Now which frame are we on?
          skip_frames = (int) cap.get(CV_CAP_PROP_POS_FRAMES);
        }

        show_preview = false;
    }

    //If it doesn't work, stop
    if(!cap.isOpened())
    {
        cerr << "Unable to open video capture." << endl;
        return -1;
    }

    Mat im0;
    
    // get init rect for ipcamera mode
    if(ipcamera_flag)
    {

    }
    else
    {
        //Show preview until key is pressed
        while (show_preview)
        {
            Mat preview;
            cap >> preview;

            screenLog(preview, "Press a key to start selecting an object.");
            imshow(WIN_NAME, preview);

            char k = waitKey(10);
            if (k != -1) {
                show_preview = false;
            }
        }

        //Get initial image
        cap >> im0;

        //If no bounding was specified, get it from user
        if (!bbox_flag)
        {
            rect = getRect(im0, WIN_NAME);
        }

        FILE_LOG(logINFO) << "Using " << rect.x << "," << rect.y << "," << rect.width << "," << rect.height
            << " as initial bounding box.";
    }

    //Convert im0 to grayscale
    Mat im0_gray;
    if (im0.channels() > 1) {
        cvtColor(im0, im0_gray, CV_BGR2GRAY);
    } else {
        im0_gray = im0;
    }

    //Initialize CMT
    cmt.initialize(im0_gray, rect);

    int frame = skip_frames;

    //Open output file.
    ofstream output_file;

    if (output_flag)
    {
        int msecs = (int) cap.get(CV_CAP_PROP_POS_MSEC);

        output_file.open(output_path.c_str());
        output_file << OUT_FILE_COL_HEADERS << endl;
        output_file << frame << "," << msecs << ",";
        output_file << cmt.points_active.size() << ",";
        output_file << write_rotated_rect(cmt.bb_rot) << endl;
    }
	double timetest = cv::getTickCount();
    //Main loop
    while (true)
    {
        frame++;

        Mat im;

        //If loop flag is set, reuse initial image (for debugging purposes)
        if (loop_flag) im0.copyTo(im);
        else cap >> im; //Else use next image in stream

        if (im.empty()) break; //Exit at end of video stream

        Mat im_gray;
        if (im.channels() > 1) {
            cvtColor(im, im_gray, CV_BGR2GRAY);
        } else {
            im_gray = im;
        }

        //Let CMT process the frame
        cmt.processFrame(im_gray);

        //Output.
        if (output_flag)
        {
            int msecs = (int) cap.get(CV_CAP_PROP_POS_MSEC);
            output_file << frame << "," << msecs << ",";
            output_file << cmt.points_active.size() << ",";
            output_file << write_rotated_rect(cmt.bb_rot) << endl;
        }
        else
        {
            //TODO: Provide meaningful output
            FILE_LOG(logINFO) << "#" << frame << " active: " << cmt.points_active.size();
        }



        //Display image and then quit if requested.
		double timetest_new = cv::getTickCount();
		double fps = cv::getTickFrequency() / (timetest_new - timetest);
		timetest = timetest_new;
		cv::putText(im, "fps : " + std::to_string(static_cast<int>(fps)), cv::Point(20, 20), 1, 1, cv::Scalar(0, 255, 0), 1);
        // display for ipcamera mode
        if(ipcamera_flag)
        {

        }
        else
        {
            char key = display(im, cmt);
            if(key == 'q') break;
        }
    }

    //Close output file.
    if (output_flag) output_file.close();

    return 0;
}
