
#pragma once

#include<iostream>
#include <ctime>
#include <string>
#include <stdlib.h>
#include<vector>
#include<stdio.h>
#include<Windows.h>
#include <omp.h>
#include <math.h> 

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracking.hpp>



/*

ver 1.21:
increased speed



*/



namespace tracker
{
	Mat element3 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3), Point(-1, -1));//3*3��ģ��
	Mat element5 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5), Point(-1, -1));//5*5��ģ��

	////////////////////////////////////////////////////////////////////////

	vector<Point2f> GetCoordinate(Mat& frame, int lower, int upper, bool draw = 0)//����������ֵ draw ���Ƿ�������
	{



		Mat Rframe; //�˺����õ�ͨ������

		cvtColor(frame, Rframe, COLOR_BGR2GRAY);

		inRange(Rframe, Scalar(lower), Scalar(upper), Rframe); // ��ֵ��ͼ��

		dilate(Rframe, Rframe, element3, Point(-1, -1), 1);
		erode(Rframe, Rframe, element5, Point(-1, -1), 1);
		dilate(Rframe, Rframe, element3, Point(-1, -1), 1);// �˳����


		vector<vector<Point>> lunkuo;//����������
		vector<Vec4i> guanxi;//������Ĺ�ϵ
		Point2f fourPoints[4];// �ž����ĸ������������

							  //imshow("test",Rframe);
							  //waitKey(10000);


		findContours(Rframe, lunkuo, guanxi, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));//�ҵ�Ŀ������

																								  //drawContours(frame,lunkuo,-1,Scalar(a,0,0));
		int tarNum = lunkuo.size();




		if (tarNum > 4)
		{
			cout << "too much target" << endl;
			tarNum = 4;
		}

		vector<Point2f> zuobiao(tarNum, Point2f(0, 0));//���ε����������

		if (tarNum == 0)
		{
			cout << "no target" << endl;
			return zuobiao;
		}




		for (int i = 0; i < tarNum; i++)
		{

			RotatedRect fangxing = minAreaRect(lunkuo[i]);   //
			fangxing.points(fourPoints);//�ĸ�������Ž�������

			zuobiao[i].x = (fourPoints[0].x + fourPoints[2].x) / 2;
			zuobiao[i].y = (fourPoints[0].y + fourPoints[2].y) / 2;//�ֱ��������


			if (draw)
			{

				for (int i = 0; i < 3; i++)
				{
					line(frame, fourPoints[i], fourPoints[i + 1], Scalar(0, 0, 255), 2);
				}

				line(frame, fourPoints[0], fourPoints[3], Scalar(0, 0, 255), 2);
			}


		}



		return zuobiao;

	}


}



namespace Pred
{

	struct Obj_Pos_Frame
	{
		Point3d Pos;

		double TimeStamp = -1;
	};

	struct Pred_Frame
	{
		double val;
		double TimeStamp;
	};
	/*sort ר��compare����*/
	static bool ccompare(Obj_Pos_Frame& a1, Obj_Pos_Frame& a2)
	{
		return (a1.TimeStamp > a2.TimeStamp);
	}

	static bool P_ccompare(Pred_Frame& a1, Pred_Frame& a2)
	{
		return (a1.TimeStamp > a2.TimeStamp);
	}



	class Predictor
	{
	public:
		Predictor(uint polyDegree = 3, uint max_frame = 10)
		{
			poly_Order = polyDegree;
			maxFrame = max_frame;
		}
		void upDate_OneFrame(Pred_Frame& NewPos)
		{
			{
				lock_guard<mutex> lck(upDate_Locker);
				//0����ʱ���ǰ�ģ���಻����max��
				Position.push_back(NewPos);
				std::sort(Position.begin(), Position.end(), P_ccompare);//��ʱ�����������
				while (Position.size() > maxFrame) Position.pop_back();//ȷ����������������
			}
			FreshData();
		}
		void Data_clear()
		{
			Position.clear();
		}

		/*
		���ض���ʽ���� ax^0+bx^1.........
		ֻ�������޸�
		*/
		void ReturnPolyFunc(double*& poly, int& degree)
		{

			poly = Pred_Poly;
			degree = poly_Order;
		}
		/*���ض���ʽ���� ax^0+bx^1.........*/
		vector<double> ReturnPolyFunc()
		{
			vector<double> poly;
			for (int a = 0; a <= poly_Order; a++)poly.push_back(Pred_Poly[a]);
			return poly;
		}


		double Position_now(double now)
		{
			if (Position.size() == 1) return Position[0].val;
			else if (Position.size() == 0) return 0;//����

			

			lock_guard<mutex> lck(poly_Locker);
			double x = 0;

			for (uint n = 0; n <= poly_Order; n++)x += Pred_Poly[n] * pow(now, n);//��ֵ

			return x;
		}
	private:

		Point3d p1;//��ǰ���һ��ˢ�µ�λ������

		mutex upDate_Locker;//���߳��ϴ���Ϣʱ�Ļ�����
		mutex poly_Locker;//���¶���ʽʱ��

		uint poly_Order = 3;//����ʽ����
		uint maxFrame = 10;//��󴢴��֡��
		std::vector<Pred_Frame> Position;//������������������

		double Pred_Poly[12] = { 0 };


		void FreshData()
		{



			double L_Pred_Poly_X[20] = {0};
			double X[20] = { 0 };

			double Time_stamp[20] = { 0 };



			uint Size = Position.size();

			for (int num = 0; num < Size; num++)
			{
				X[Size - num - 1] = Position[num].val;
				Time_stamp[Size - num - 1] = Position[num].TimeStamp;
			}


			//lock_guard<mutex> lck(poly_Locker);


				polyfit(Size, Time_stamp, X, poly_Order, L_Pred_Poly_X);


			lock_guard<mutex> lck(poly_Locker);
			for (int n = 0; n <= poly_Order; n++)Pred_Poly[n] = L_Pred_Poly_X[n];


		}

		void polyfit(int n, double x[], double y[], int poly_n, double a[])
		{
			int i, j;
			double* tempx, * tempy, * sumxx, * sumxy, * ata;
			void gauss_solve(int n, double A[], double x[], double b[]);
			tempx = (double*)calloc(n, sizeof(double));
			sumxx = (double*)calloc(poly_n * 2 + 1, sizeof(double));
			tempy = (double*)calloc(n, sizeof(double));
			sumxy = (double*)calloc(poly_n + 1, sizeof(double));
			ata = (double*)calloc((poly_n + 1) * (poly_n + 1), sizeof(double));
			for (i = 0; i < n; i++)
			{
				tempx[i] = 1;
				tempy[i] = y[i];
			}
			for (i = 0; i < 2 * poly_n + 1; i++)
				for (sumxx[i] = 0, j = 0; j < n; j++)
				{
					sumxx[i] += tempx[j];
					tempx[j] *= x[j];
				}
			for (i = 0; i < poly_n + 1; i++)
				for (sumxy[i] = 0, j = 0; j < n; j++)
				{
					sumxy[i] += tempy[j];
					tempy[j] *= x[j];
				}
			for (i = 0; i < poly_n + 1; i++)
				for (j = 0; j < poly_n + 1; j++)
					ata[i * (poly_n + 1) + j] = sumxx[i + j];
			Gauss_solve(poly_n + 1, ata, a, sumxy);

			free(tempx);
			free(sumxx);
			free(tempy);
			free(sumxy);
			free(ata);
		}
		void Gauss_solve(int n, double* A, double* x, double* b)
		{
			int i, j, k, r;
			double max;
			for (k = 0; k < n - 1; k++)
			{
				max = fabs(A[k * n + k]); /*find maxmum*/
				r = k;
				for (i = k + 1; i < n - 1; i++)
					if (max < fabs(A[i * n + i]))
					{
						max = fabs(A[i * n + i]);
						r = i;
					}
				if (r != k)
					for (i = 0; i < n; i++)         /*change array:A[k]&A[r] */
					{
						max = A[k * n + i];
						A[k * n + i] = A[r * n + i];
						A[r * n + i] = max;
					}
				max = b[k];                    /*change array:b[k]&b[r]     */
				b[k] = b[r];
				b[r] = max;
				for (i = k + 1; i < n; i++)
				{
					for (j = k + 1; j < n; j++)
						A[i * n + j] -= A[i * n + k] * A[k * n + j] / A[k * n + k];
					b[i] -= A[i * n + k] * b[k] / A[k * n + k];
				}
			}

			for (i = n - 1; i >= 0; x[i] /= A[i * n + i], i--)
				for (j = i + 1, x[i] = b[i]; j < n; j++)
					x[i] -= A[i * n + j] * x[j];
			return;
		}
	};
	

	class Obj_Predictor
	{
	public:
		Obj_Predictor(uint polyDegree = 3, uint max_frame = 10)
		{
			poly_Order = polyDegree;
			maxFrame = max_frame;
		}
		void upDate_OneFrame(Obj_Pos_Frame& NewPos)
		{
			{
				lock_guard<mutex> lck(upDate_Locker);
				//0����ʱ���ǰ�ģ���಻����max��
				Position.push_back(NewPos);
				std::sort(Position.begin(), Position.end(), ccompare);
				while (Position.size() > maxFrame) Position.pop_back();
			}
			FreshData();
		}
		void Data_clear()
		{
			Position.clear();
		}


		Point3d Position_now(double now)
		{
			if (Position.size() == 1) return Position[0].Pos;
			else if (Position.size() == 0) return Point3d(0, 0, 0);//����

			double x = 0, y = 0, z = 0;

			lock_guard<mutex> lck(poly_Locker);

#pragma omp parallel sections
			{
#pragma omp section
				for (uint n = 0; n <= poly_Order; n++)x += Pred_Poly_X[n] * pow(now, n);
#pragma omp section
				for (uint n = 0; n <= poly_Order; n++)y += Pred_Poly_Y[n] * pow(now, n);
#pragma omp section
				for (uint n = 0; n <= poly_Order; n++)z += Pred_Poly_Z[n] * pow(now, n);
			}
			p1.x = x;
			p1.y = y;
			p1.z = z;
			return p1;//��ȥ�ظ�����ʵ��
		}
	private:

		Point3d p1;//��ǰ���һ��ˢ�µ�λ������

		mutex upDate_Locker;//���߳��ϴ���Ϣʱ�Ļ�����
		mutex poly_Locker;//���¶���ʽʱ��

		uint poly_Order = 3;//����ʽ����
		uint maxFrame = 10;//��󴢴��֡��
		std::vector<Obj_Pos_Frame> Position;//������������������

		double Pred_Poly_X[12] = { 0 };
		double Pred_Poly_Y[12] = { 0 };
		double Pred_Poly_Z[12] = { 0 };

		void FreshData()
		{

			double L_Pred_Poly_X[12] = { 0 };
			double L_Pred_Poly_Y[12] = { 0 };
			double L_Pred_Poly_Z[12] = { 0 };

			

			uint Size = Position.size();
			double X[10] = { 0 };
			double Y[10] = { 0 };
			double Z[10] = { 0 };
			double Time_stamp[10] = { 0 };
			for (int num = 0; num < Size; num++)
			{
				Obj_Pos_Frame& P = Position[num];
				X[Size - num - 1] = P.Pos.x;
				Y[Size - num - 1] = P.Pos.y;
				Z[Size - num - 1] = P.Pos.z;
				Time_stamp[Size - num - 1] = P.TimeStamp;
			}


			//lock_guard<mutex> lck(poly_Locker);

#pragma omp parallel sections
			{
#pragma omp section
				polyfit(Size, Time_stamp, X, poly_Order, L_Pred_Poly_X);
#pragma omp section
				polyfit(Size, Time_stamp, Y, poly_Order, L_Pred_Poly_Y);
#pragma omp section
				polyfit(Size, Time_stamp, Z, poly_Order, L_Pred_Poly_Z);
			}
			
			lock_guard<mutex> lck(poly_Locker);
			for (int n = 0; n <= poly_Order; n++)Pred_Poly_X[n] = L_Pred_Poly_X[n];
			for (int n = 0; n <= poly_Order; n++)Pred_Poly_Y[n] = L_Pred_Poly_Y[n];
			for (int n = 0; n <= poly_Order; n++)Pred_Poly_Z[n] = L_Pred_Poly_Z[n];

		}

		void polyfit(int n, double x[], double y[], int poly_n, double a[])
		{
			int i, j;
			double* tempx, * tempy, * sumxx, * sumxy, * ata;
			void gauss_solve(int n, double A[], double x[], double b[]);
			tempx = (double*)calloc(n, sizeof(double));
			sumxx = (double*)calloc(poly_n * 2 + 1, sizeof(double));
			tempy = (double*)calloc(n, sizeof(double));
			sumxy = (double*)calloc(poly_n + 1, sizeof(double));
			ata = (double*)calloc((poly_n + 1) * (poly_n + 1), sizeof(double));
			for (i = 0; i < n; i++)
			{
				tempx[i] = 1;
				tempy[i] = y[i];
			}
			for (i = 0; i < 2 * poly_n + 1; i++)
				for (sumxx[i] = 0, j = 0; j < n; j++)
				{
					sumxx[i] += tempx[j];
					tempx[j] *= x[j];
				}
			for (i = 0; i < poly_n + 1; i++)
				for (sumxy[i] = 0, j = 0; j < n; j++)
				{
					sumxy[i] += tempy[j];
					tempy[j] *= x[j];
				}
			for (i = 0; i < poly_n + 1; i++)
				for (j = 0; j < poly_n + 1; j++)
					ata[i * (poly_n + 1) + j] = sumxx[i + j];
			Gauss_solve(poly_n + 1, ata, a, sumxy);

			free(tempx);
			free(sumxx);
			free(tempy);
			free(sumxy);
			free(ata);
		}
		void Gauss_solve(int n, double* A, double* x, double* b)
		{
			int i, j, k, r;
			double max;
			for (k = 0; k < n - 1; k++)
			{
				max = fabs(A[k * n + k]); /*find maxmum*/
				r = k;
				for (i = k + 1; i < n - 1; i++)
					if (max < fabs(A[i * n + i]))
					{
						max = fabs(A[i * n + i]);
						r = i;
					}
				if (r != k)
					for (i = 0; i < n; i++)         /*change array:A[k]&A[r] */
					{
						max = A[k * n + i];
						A[k * n + i] = A[r * n + i];
						A[r * n + i] = max;
					}
				max = b[k];                    /*change array:b[k]&b[r]     */
				b[k] = b[r];
				b[r] = max;
				for (i = k + 1; i < n; i++)
				{
					for (j = k + 1; j < n; j++)
						A[i * n + j] -= A[i * n + k] * A[k * n + j] / A[k * n + k];
					b[i] -= A[i * n + k] * b[k] / A[k * n + k];
				}
			}

			for (i = n - 1; i >= 0; x[i] /= A[i * n + i], i--)
				for (j = i + 1, x[i] = b[i]; j < n; j++)
					x[i] -= A[i * n + j] * x[j];
			return;
		}
	};

	/*���ض���ʽ���� ax^0+bx^1.........*/
	vector<double> poly_derivative(const vector<double>& input)
	{
		vector<double> derivative;
		for (int a = 1; a < input.size(); a++)derivative.push_back(a * input[a]);
		return derivative;
	}
	/*���ض���ʽ���� ax^0+bx^1.........*/
	vector<double> poly_integration(const vector<double>& input,double C=0.0)
	{
		
		vector<double> integration;
		integration.push_back(C);
		for (int a = 0; a < input.size(); a++)integration.push_back(input[a]/(a+1));
		return integration;
	}

	double poly_val(const vector<double>& input, double x)
	{
		double y=0;
		for (int d = 0; d < input.size(); d++)y += input[d] * pow(x,d);
		return y;
	}


}


