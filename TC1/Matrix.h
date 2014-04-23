/* 
 * Last Updated at [2014/4/23 10:38] by wuhao
 */
#pragma once
#include <iostream>
using namespace std;

template<class T>
class Matrix
{
public:
	Matrix(int row, int col);
	Matrix(const Matrix<T>& otherMat);
	T& at(int row, int col);
	T& at(int row, int col) const;
	Matrix<T>& operator=(const Matrix<T>& rhs);
	Matrix<T> operator+(const Matrix<T>& rhs) const;
	Matrix<T> operator-(const Matrix<T>& rhs) const;
	Matrix<T> operator*(const Matrix<T>& rhs) const;
	Matrix<T> traspose();
	void print();
//private:
	T* mat;
	int row, col;
};

template<class T>
Matrix<T>::Matrix(int row, int col)
{
	mat = new T[row * col];
	this->row = row;
	this->col = col;
	//清零
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			mat[row * i + j] = 0;
		}
	}
}

template<class T>
Matrix<T>::Matrix(const Matrix<T>& otherMat)
{
	this->row = otherMat.row;
	this->col = otherMat.col;
	this->mat = new T[row * col];
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			this->mat[row * i + j] = otherMat.mat[row * i + j];
		}
	}
}

template<class T>
inline T& Matrix<T>::at(int row, int col)
{
	return mat[row * this->row + col];
}

template<class T>
inline T& Matrix<T>::at(int row, int col) const
{
	return mat[row * this->row + col];
}

template<class T>
Matrix<T>& Matrix<T>::operator=(const Matrix<T>& rhs)
{
	row = rhs.row;
	col = rhs.col;
	mat = new T[row * col];
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			at(i, j) = rhs.at(i, j);
		}
	}
	return *this;
}

template<class T>
Matrix<T> Matrix<T>::operator+(const Matrix<T>& rhs) const
{
	if (this->row != rhs.row || this->col != rhs.col)
	{
		cout << "矩阵不可相加！" << endl;
		system("pause");
		exit(0);
	}
	Matrix<T> tempMat(this->row, this->col);
	for (int i = 0; i < this->row; i++)
	{
		for (int j = 0; j < this->col; j++)
		{
			tempMat.at(i, j) = this->at(i, j) + rhs.at(i, j);
		}
	}
	return tempMat;
}

template<class T>
Matrix<T> Matrix<T>::operator-(const Matrix<T>& rhs) const
{
	if (this->row != rhs.row || this->col != rhs.col)
	{
		cout << "矩阵不可相减！" << endl;
		system("pause");
		exit(0);
	}
	Matrix<T> tempMat(this->row, this->col);
	for (int i = 0; i < this->row; i++)
	{
		for (int j = 0; j < this->col; j++)
		{
			tempMat.at(i, j) = this->at(i, j) - rhs.at(i, j);
		}
	}
	return tempMat;
}


template<class T>
Matrix<T> Matrix<T>::operator*(const Matrix<T>& rhs) const
{
	if (this->col != rhs.row)
	{
		cout << "矩阵不可相乘！" << endl;
		system("pause");
		exit(0);
	}
	Matrix<T> tempMat(this->row, rhs.col);
	for (int i = 0; i < this->row; i++)
	{
		for (int j = 0; j < rhs.col; j++)
		{
			T value = 0;
			for (int k = 0; k < this->col; k++)
			{
				value += this->at(i, k) * rhs.at(k, j);
			}
			tempMat.at(i, j) = value;
		}
	}
	return tempMat;
}

template<class T>
Matrix<T> Matrix<T>::traspose()
{
	Matrix<T> tempMat(col, row);
	for (int i = 0; i < col; i++)
	{
		for (int j = 0; j < row; j++)
		{
			tempMat.at(i, j) = this->at(j, i);
		}
	}
	return tempMat;
}

template<class T>
void Matrix<T>::print()
{
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			cout << at(i, j) << "\t";
		}
		cout << endl;
	}
}