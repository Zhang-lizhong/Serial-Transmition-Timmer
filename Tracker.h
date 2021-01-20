
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
	Mat element3 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3), Point(-1, -1));//3*3的模板
	Mat element5 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5), Point(-1, -1));//5*5的模板

	////////////////////////////////////////////////////////////////////////

	vector<Point2f> GetCoordinate(Mat& frame, int lower, int upper, bool draw = 0)//后两个是阈值 draw 是是否绘出轮廓
	{



		Mat Rframe; //此函数用单通道矩阵

		cvtColor(frame, Rframe, COLOR_BGR2GRAY);

		inRange(Rframe, Scalar(lower), Scalar(upper), Rframe); // 二值化图像

		dilate(Rframe, Rframe, element3, Point(-1, -1), 1);
		erode(Rframe, Rframe, element5, Point(-1, -1), 1);
		dilate(Rframe, Rframe, element3, Point(-1, -1), 1);// 滤除噪点


		vector<vector<Point>> lunkuo;//轮廓的容器
		vector<Vec4i> guanxi;//轮廓间的关系
		Point2f fourPoints[4];// 放矩形四个点坐标的容器

							  //imshow("test",Rframe);
							  //waitKey(10000);


		findContours(Rframe, lunkuo, guanxi, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));//找到目标轮廓

																								  //drawContours(frame,lunkuo,-1,Scalar(a,0,0));
		int tarNum = lunkuo.size();




		if (tarNum > 4)
		{
			cout << "too much target" << endl;
			tarNum = 4;
		}

		vector<Point2f> zuobiao(tarNum, Point2f(0, 0));//矩形的坐标的向量

		if (tarNum == 0)
		{
			cout << "no target" << endl;
			return zuobiao;
		}




		for (int i = 0; i < tarNum; i++)
		{

			RotatedRect fangxing = minAreaRect(lunkuo[i]);   //
			fangxing.points(fourPoints);//四个点坐标放进容器内

			zuobiao[i].x = (fourPoints[0].x + fourPoints[2].x) / 2;
			zuobiao[i].y = (fourPoints[0].y + fourPoints[2].y) / 2;//分别放入坐标


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
	/*sort 专用compare函数*/
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
				//0是最时间最靠前的，最多不超过max个
				Position.push_back(NewPos);
				std::sort(Position.begin(), Position.end(), P_ccompare);//以时间戳进行排序
				while (Position.size() > maxFrame) Position.pop_back();//确保对象数量不超标
			}
			FreshData();
		}
		void Data_clear()
		{
			Position.clear();
		}

		/*
		返回多项式函数 ax^0+bx^1.........
		只读不能修改
		*/
		void ReturnPolyFunc(double*& poly, int& degree)
		{

			poly = Pred_Poly;
			degree = poly_Order;
		}
		/*返回多项式函数 ax^0+bx^1.........*/
		vector<double> ReturnPolyFunc()
		{
			vector<double> poly;
			for (int a = 0; a <= poly_Order; a++)poly.push_back(Pred_Poly[a]);
			return poly;
		}


		double Position_now(double now)
		{
			if (Position.size() == 1) return Position[0].val;
			else if (Position.size() == 0) return 0;//防呆

			

			lock_guard<mutex> lck(poly_Locker);
			double x = 0;

			for (uint n = 0; n <= poly_Order; n++)x += Pred_Poly[n] * pow(now, n);//算值

			return x;
		}
	private:

		Point3d p1;//当前最近一次刷新的位置数据

		mutex upDate_Locker;//多线程上传信息时的互斥锁
		mutex poly_Locker;//更新多项式时的

		uint poly_Order = 3;//多项式阶数
		uint maxFrame = 10;//最大储存的帧数
		std::vector<Pred_Frame> Position;//储存坐标数据主容器

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
				//0是最时间最靠前的，最多不超过max个
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
			else if (Position.size() == 0) return Point3d(0, 0, 0);//防呆

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
			return p1;//免去重复创建实例
		}
	private:

		Point3d p1;//当前最近一次刷新的位置数据

		mutex upDate_Locker;//多线程上传信息时的互斥锁
		mutex poly_Locker;//更新多项式时的

		uint poly_Order = 3;//多项式阶数
		uint maxFrame = 10;//最大储存的帧数
		std::vector<Obj_Pos_Frame> Position;//储存坐标数据主容器

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

	/*返回多项式函数 ax^0+bx^1.........*/
	vector<double> poly_derivative(const vector<double>& input)
	{
		vector<double> derivative;
		for (int a = 1; a < input.size(); a++)derivative.push_back(a * input[a]);
		return derivative;
	}
	/*返回多项式函数 ax^0+bx^1.........*/
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
	
	
	
double cubr(double x)
{
	if (x > 0)return pow(x,1/3.0);
	return -pow(-x,1/3.0);
}
complex<double> cubr(complex<double> x)
{
	if ((abs(x.imag()) < 1e-10) && (x.real() < 0))return complex<double>(-pow(-x.real(),1/3.0));

	else return pow(x, 1/3.0);
}
complex<double> sqroot(complex<double> x)
{
	if ((abs(x.imag()) < 1e-10) && (x.real() < 0))return complex<double>(-pow(-x.real(), 0.5));

	else return pow(x, 0.5);
}

vector <double> solveCubic(double a, double b,  double c,  double d)
{
	/*https://zhuanlan.zhihu.com/p/40349993 */

	typedef complex< double> C;

	 double p, q;

	p = (3 * a * c - b * b) / (3 * a * a);
	q = ((27 * a * a * d) - (9 * a * b * c) + 2 * pow(b, 3)) / (27 * pow(a, 3));



	double delta=q*q/4+pow(p/3,3);

	//cout << delta << endl;
	


	C rootDelta = sqroot(C(delta));


	C plus_root = cubr(C(-q / 2, 0) + rootDelta);
	C minu_root = cubr(C(-q / 2, 0) - rootDelta);


	C omega(-0.5,0.8660254037844386468);
	C omega2 = omega * omega;


	C y_x(b / (3 * a));

	C x1 = plus_root + minu_root - y_x;
	C x2 = (omega * plus_root) + (omega2 * minu_root) - y_x;
	C x3 = (omega2 * plus_root) + (omega * minu_root) - y_x;

	vector <double> output;

	if (abs(x1.imag()) < 1e-10)output.push_back(x1.real());
	if (abs(x2.imag()) < 1e-10)output.push_back(x2.real());
	if (abs(x3.imag()) < 1e-10)output.push_back(x3.real());

	//double X1 = cubr(-q / 2 + sqrt(delta)) + cubr(-q / 2 - sqrt(delta))-(b/(3*a));



	//cout << cubr(-q / 2 - sqrt(delta)) << endl;
	//cout << minu_root << endl;
	////cout << X1 << endl;
	//cout << x1 << endl;
	//cout << x2 << endl;
	//cout << x3 << endl;

	return output;
//	cout << y2 << endl;
//	cout << y3 << endl;

}

vector <double> predPoly(double F,double S,double y2,double Y1,double dt)
{
	double& C = Y1;
	vector <double> x;
	{
		double a = (F / 4 - F / 6);
		double b = 0;
		double c = (-C / 2 - y2 / 2);
		double d = S;

		x= solveCubic(a, b, c, d);
	}

	vector<double> b;//real b


	
	for (int num=0; num<x.size(); num++)
	{
		double X = x[num];
		double B = (y2 - C - (F / 2) * X * X)/X ;
		
		double result = (F / 6) * pow(X, 3) + (B / 2) * pow(X, 2) + C * X;

		if (abs(result - S) < 1e-10)b.push_back(B);
	}

	//cout << b.size()<<endl;
	sort(b.begin(),b.end());
	
	return vector<double>{(dt*(F/2)-b[0])*dt+C,b[0]-dt*F,F/2};
	//return vector<double>{C, b[0], F / 2};
}


}


