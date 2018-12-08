#include <opencv2/opencv.hpp>
#include  <iostream>

using namespace std;
using namespace cv;

//ȫ�ֱ���
Mat src, gray, gray_blur, contours_image,dstThreshold;
//������ֱ���Ϊ640*480������ͼ������Ϊͼ��ԭ�㣬Ҳ����Ӧ���������λ�ã�����λ��
//�Զ���ͼ��ԭ������
float oriX = 296.0f;
float oriY = 241.0f;

float targetImage_X, targetImage_Y;  //Ŀ�����ͼ��ԭ���X,Y��������ؾ���
float mm_per_pixel;                  //���سߴ�
float targetLength=10.45f;           //Ŀ����ʵ�ʳ���
float targetActualX, targetActualY;  //��ά�ռ�ʵ������

//�Ӿ���λ����
void Location();

int main()
{
	cout <<" ������������������������������������'q'�˳����򡣡���������������������������" << endl;
	VideoCapture cap(0);
	if (!cap.isOpened()){
	    cout << "Failed To Open capture";
		return -1;
	}
	while (1){
		cap >> src;
		cvtColor(src, gray, CV_BGR2GRAY);
		imshow("gray", gray);
		
		contours_image = Mat::zeros(src.rows, src.cols, CV_8UC3);
		gray_Contrast = Mat::zeros(gray.size(), gray.type());

		Location();
		char key = waitKey(10);
		if (key == 'q'){
			break;
		}
	}
}


//��λ����
void Location(){

	//��ֵ�˲�
	blur(gray, gray_blur, Size(3, 3));

	//��Ե�����ȡ��Ե��Ϣ
	Canny(gray_blur, dstThreshold, 150, 450);
	imshow("canny��Ե���", dstThreshold);
	
	//�Ա�Եͼ����ȡ������Ϣ
    vector<vector<Point> >contours;
	findContours(dstThreshold, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	
	//��������
	drawContours(contours_image, contours, -1, Scalar(0, 0, 255));
	imshow("contours", contours_image);

	//���������ԭ��
	circle(src, Point2f(oriX, oriY), 2, Scalar(0, 0, 255), 3);

	//����ֱ�ƽ������κ�����ε�����
	vector< vector<Point> > Contour1_Ok, Contour2_Ok;

	//��������
	vector<Point> approx;
	for (int i = 0; i < contours.size(); i++){
		approxPolyDP(Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.04, true);

		//ȥ�� С������ֻ��ȡ͹����
		if (std::fabs(cv::contourArea(contours[i])) < 600 || !cv::isContourConvex(approx))
			continue;

	    //����ƽ������ε����� �� Contour1_Ok
		 if (approx.size() == 6){
			Contour1_Ok.push_back(contours[i]);
		}
	    //����ƽ�����ε����� �� Contour2_Ok
		 else if (approx.size() == 5){
			 Contour2_Ok.push_back(contours[i]);
		}

	}

	//�����з���Ҫ��������Σ�������������з���
	//ʶ����Զ��������Ĺؼ��ǣ�
	//1.�����κ��������������С��Ӿ��ε����Ļ�����ͬһ��
	//2.��������������С��Ӿ��ε���һ�߳������������������С��Ӿ��ε���һ�߳�
	for (int i = 0; i < Contour1_Ok.size(); i++){
		for (int j = 0; j < Contour2_Ok.size(); j++){
			RotatedRect minRect1 = minAreaRect(Mat(Contour1_Ok[i]));  //��������������С��Ӿ���
			RotatedRect minRect2 = minAreaRect(Mat(Contour2_Ok[j]));  //�������������С������
			//�ҳ�����Ҫ�����������С��Ӿ���
			if ( fabs(minRect1.center.x - minRect2.center.x) < 30 && fabs(minRect1.center.y - minRect2.center.y)<30 && minRect1.size.width > minRect2.size.width){
				Point2f vtx[4];
				minRect1.points(vtx);
				
				//�����ҵ����������С��Ӿ���
				for (int j = 0; j < 4; j++)
					line(src, vtx[j], vtx[(j + 1) % 4], Scalar(0, 0, 255), 2, LINE_AA);

				//����Ŀ�������ĵ�ͼ��ԭ���ֱ��
				line(src, minRect1.center, Point2f(oriX, oriY), Scalar(0, 255, 0), 1, LINE_AA);

				//Ŀ�����ͼ��ԭ���X,Y��������ؾ���
				targetImage_X = minRect1.center.x - oriX;
				targetImage_Y = oriY - minRect1.center.y;

				line(src, minRect1.center, Point2f(minRect1.center.x, oriY), Scalar(255, 0, 0), 1, LINE_AA);
				line(src, Point2f(oriX, oriY), Point2f(minRect1.center.x, oriY), Scalar(255, 0, 0), 1, LINE_AA);

                Point2f pointX((oriX + minRect1.center.x) / 2, oriY);
				Point2f pointY(minRect1.center.x, (oriY + minRect1.center.y) / 2);  
		
		        //�ҳ�����
				float a = minRect1.size.height, b = minRect1.size.width;
				if (a < b) a = b;
				
				mm_per_pixel = targetLength / a;               //�������سߴ� = Ŀ�����ʵ�ʳ��ȣ�cm��/ Ŀ������ͼ���ϵ����س��ȣ�pixels��
				targetActualX = mm_per_pixel *targetImage_X;   //����ʵ�ʾ���X��cm��
				targetActualY = mm_per_pixel *targetImage_Y;   //����ʵ�ʾ���Y��cm��

				//��ӡ��Ϣ��ͼƬ��
				String text1 = "X:"+format("%f", targetImage_X);
				String text2 = "Y:"+format("%f", targetImage_Y);
				putText(src, text1, pointX, FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 1, 8);
				putText(src, text2, pointY, FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 1, 8);

				String text3 = "Target_X:"+format("%f", targetActualX);
				String text4 = "Target_Y:"+format("%f", targetActualY);
				putText(src, text3, Point(10,30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1, 8);
				putText(src, text4, Point(10,60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 1, 8);

				
			}
			break;
		}       
        break;
	}

	imshow("SRC", src);

}