#include "EigenMatrixOperate.h"

MatrixXd abs(MatrixXd mat)
{
	MatrixXd matout(mat.rows(), mat.cols());
	for (int i = 0; i < mat.rows(); i++)
	{
		for (int j = 0; j < mat.cols(); j++)
		{
			matout(i, j) = abs(mat(i, j));
		}
	}
	return matout;
}

MatrixXd find(MatrixXd Input, string Operator, double num)
{
	//cout << Input << endl;
	
	MatrixXd Index(Input.rows()*Input.cols(),1);
	int IndexNum = 0;
	for (int i = 0; i < Input.rows()*Input.cols(); i++)
	{
		if (Operator == "<")
		{
			if (Input(i) < num)
			{
				Index(IndexNum) = i;
				IndexNum++;
			}
		}
		else if (Operator == "<=")
		{
			if (Input(i) <= num)
			{
				Index(IndexNum) = i;
				IndexNum++;
			}
		}
		else if (Operator == ">")
		{
			if (Input(i) > num)
			{
				Index(IndexNum) = i;
				IndexNum++;
			}
		}
		else if (Operator == ">=")
		{
			if (Input(i) >= num)
			{
				Index(IndexNum) = i;
				IndexNum++;
			}
		}
		else if (Operator == "==")
		{
			if (Input(i) == num)
			{
				Index(IndexNum) = i;
				IndexNum++;
			}
		}
		else
			error("find函数目前暂时没有定义这样的符号进行计算");

	}
	//cout << Index << endl;
	MatrixXd tmp = Index.block(0, 0, IndexNum, 1);
	Index = tmp;
	//cout << Index << endl;
	return Index;
}

MatrixXd addMatrix2Matrix(MatrixXd ResultMatrix, string Operator,MatrixXd addMatrix)
//如无必要，最好不要用这个函数，速度慢 最好先预先分配 内存
{
	if (ResultMatrix.rows() == 0 || ResultMatrix.cols() == 0)
	{
		ResultMatrix = addMatrix;
		return ResultMatrix;
	}
	if (addMatrix.rows() == 0 || addMatrix.cols() == 0)
	{
		return ResultMatrix;
	}

	if (Operator == ";")
	{
		if(ResultMatrix.cols()!= addMatrix.cols())
			error("按列连接的两矩阵的维度不同，即列数不同");
		MatrixXd tmp(ResultMatrix.rows() + addMatrix.rows(), ResultMatrix.cols());
		tmp.block(0, 0, ResultMatrix.rows(), ResultMatrix.cols()) = ResultMatrix;
		tmp.block(ResultMatrix.rows(), 0, addMatrix.rows(), addMatrix.cols()) = addMatrix;
		ResultMatrix = tmp;
	}
	else if (Operator == " ")
	{
		if (ResultMatrix.rows() != addMatrix.rows())
			error("按行连接的两矩阵的维度不同，即行数不同");
		MatrixXd tmp(ResultMatrix.rows(),ResultMatrix.cols() + addMatrix.cols() );
		//cout << tmp << endl;
		tmp.block(0, 0, ResultMatrix.rows(), ResultMatrix.cols()) = ResultMatrix;
		//cout << tmp << endl;
		tmp.block(0, ResultMatrix.cols(), addMatrix.rows(), addMatrix.cols()) = addMatrix;
		//cout << tmp << endl;
		ResultMatrix = tmp;
		//cout << ResultMatrix << endl;
	}
	else
	{
		error("addMatrix2Matrix没有定义这样的操作符号");
	}
	return ResultMatrix;
}

int isempty(MatrixXd mat)
{
	if (mat.rows() == 0 || mat.cols() == 0)
		return 1;
	else
		return 0;
}