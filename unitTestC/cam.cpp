#include "opencv2/opencv.hpp"

#define EVER (;;)

using namespace cv;
using namespace std;

int main(int argc, char** argv){

	VideoCapture cap;

	if(!cap.open(0)){
		cout << "failure opening" << endl;
		return 0;
	}	

	for EVER {
		Mat image;
		cap >> image;
		if(image.empty())
			break;

		imshow("camera",image);
		
		if(waitKey(10) == 27 )
			break;
	}

	//cap.close();

	return 0;
}
