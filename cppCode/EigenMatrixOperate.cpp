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
			error("find����Ŀǰ��ʱû�ж��������ķ��Ž��м���");

	}
	//cout << Index << endl;
	MatrixXd tmp = Index.block(0, 0, IndexNum, 1);
	Index = tmp;
	//cout << Index << endl;
	return Index;
}

MatrixXd addMatrix2Matrix(MatrixXd ResultMatrix, string Operator,MatrixXd addMatrix)
//���ޱ�Ҫ����ò�Ҫ������������ٶ��� �����Ԥ�ȷ��� �ڴ� �����Ƕ���ÿ�ζ���һ��(��)������
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
			error("�������ӵ��������ά�Ȳ�ͬ����������ͬ");
		MatrixXd tmp(ResultMatrix.rows() + addMatrix.rows(), ResultMatrix.cols());
		tmp.block(0, 0, ResultMatrix.rows(), ResultMatrix.cols()) = ResultMatrix;
		tmp.block(ResultMatrix.rows(), 0, addMatrix.rows(), addMatrix.cols()) = addMatrix;
		ResultMatrix = tmp;
	}
	else if (Operator == " " || Operator == ",")
	{
		if (ResultMatrix.rows() != addMatrix.rows())
			error("�������ӵ��������ά�Ȳ�ͬ����������ͬ");
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
		error("addMatrix2Matrixû�ж��������Ĳ�������");
	}
	return ResultMatrix;
}

MatrixXd addMatrix2Matrix(double num, string Operator, MatrixXd addMatrix)
{
	if (addMatrix.rows() != 1 || addMatrix.cols() != 1)
		error("��ͬά�ȵ��������ݲ���ƴ����һ��");
	MatrixXd nummatrix(1, 1);
	nummatrix(0) = num;
	MatrixXd result;
	result = addMatrix2Matrix(nummatrix, Operator, addMatrix);
	return result;
}

MatrixXd addMatrix2Matrix(double num, string Operator, double addnum)
{
	MatrixXd addMatrix(1,1);
	addMatrix(0) = addnum;

	MatrixXd nummatrix(1, 1);
	nummatrix(0) = num;
	MatrixXd result;
	result = addMatrix2Matrix(nummatrix, Operator, addMatrix);
	return result;
}

MatrixXd addMatrix2Matrix(MatrixXd num, string Operator, double addMatrix)
{
	if (num.rows() != 1 || num.cols() != 1)
		error("��ͬά�ȵ��������ݲ���ƴ����һ��");
	MatrixXd nummatrix(1, 1);
	nummatrix(0) = addMatrix;
	MatrixXd result;
	result = addMatrix2Matrix(nummatrix, Operator, addMatrix);
	return result;
}

int isempty(MatrixXd mat)
{
	if (mat.rows() == 0 || mat.cols() == 0)
		return 1;
	else
		return 0;
}

int size(MatrixXd mat, int num)
{
	if (num == 1)
		return mat.rows();
	else if (num == 2)
		return mat.cols();
	else
		error("sizeδ������������ֺ���");
}

MatrixXd GetIntersection(MatrixXd qujianA, MatrixXd qujianB)
//�������� ����ļ���Ӧ��������� ��С����
//����Ӧ���������� ����������Ԫ�ص���������2
{
	//cout << qujianA << endl;
	//cout << qujianB << endl;
	


	//if (size(qujianA, 1) > 1 || size(qujianB, 1) > 1)
	//{
	//	error("GetIntersection�޷�������������1��");
	//}
	//	
	//if (size(qujianA, 2) > 2 || size(qujianB, 2) > 2)
	//	error("GetIntersection�޷�������������2��");

	MatrixXd out;
	if (isempty(qujianA)==1 || isempty(qujianB)==1)
	{
		out.resize(0,0);
		return out;
	}

	if (size(qujianA, 1) == 1 || size(qujianA, 2) == 1)
	{
		if (size(qujianA, 1) != 1)
		{
			MatrixXd tmp;
			tmp = qujianA.transpose();
			qujianA = tmp;
		}
	}
	else
		error("GetIntersection qujianA������");
	if (size(qujianB, 1) == 1 || size(qujianB, 2) == 1)
	{
		if (size(qujianB, 1) != 1)
		{
			MatrixXd tmp;
			tmp = qujianB.transpose();
			qujianB = tmp;
		}
	}
	else
		error("GetIntersection qujianB������");


	if (size(qujianA, 2) == 2 && qujianA(0) == qujianA(1))
	{
		double tmp;
		tmp = qujianA(0);
		qujianA.resize(1, 1);
		qujianA(0) = tmp;
	}
	if (size(qujianB, 2) == 2 && qujianB(0) == qujianB(1))
	{
		double tmp;
		tmp = qujianB(0);
		qujianB.resize(1, 1);
		qujianB(0) = tmp;
	}
	if (size(qujianA, 2) == 1 && size(qujianB, 2) == 1)
	{
		if (qujianA == qujianB)
			out = qujianA;
		else
			out.resize(0, 0);
		return out;
	}
	if (size(qujianA, 2) == 2 && qujianA(0) > qujianA(1))
	{
		double tmp = qujianA(0);
		qujianA(0) = qujianA(1);
		qujianA(1) = tmp;
	}
	if (size(qujianB, 2) == 2 && qujianB(0) > qujianB(1))
	{
		double tmp = qujianB(0);
		qujianB(0) = qujianB(1);
		qujianB(1) = tmp;
	}
	if (size(qujianA, 2) == 1 && size(qujianB, 2) == 2)
	{
		if (qujianA(0) < qujianB(0) || qujianA(0) > qujianB(1))
			out.resize(0, 0);
		else
		{
			out.resize(1, 1);
			out(0) = qujianA(0);
		}
		return out;
	}
	if (size(qujianA, 2) == 2 && size(qujianB, 2) == 1)
	{
		if (qujianB(0) < qujianA(0) || qujianB(0) > qujianA(1))
			out.resize(0, 0);
		else
		{
			out.resize(1, 1);
			out(0) = qujianB(0);
		}
		return out;
	}

	if (qujianA(1) < qujianB(0) || qujianA(0) > qujianB(1))
		out.resize(0, 0);
	else
	{
		if (qujianA(1) == qujianB(0))
		{
			out.resize(1, 1);
			out(0) = qujianA(1);
		}
			
		if (qujianA(0) == qujianB(1))
		{
			out.resize(1, 1);
			out(0) = qujianB(1);
		}
			
		if (isempty(out) == 0)
			return out;
		if (qujianA(0) <= qujianB(0))
		{
			if (qujianA(1) > qujianB(0) && qujianA(1) <= qujianB(1))
			{
				MatrixXd tmp(1, 2);
				tmp(0) = qujianB(0);
				tmp(1) = qujianA(1);
				out = tmp;
			}
			else
			{
				MatrixXd tmp(1, 2);
				tmp(0) = qujianB(0);
				tmp(1) = qujianB(1);
				out = tmp;
			}
		}
		else
		{
			if (qujianA(1) > qujianB(0) && qujianA(1) <= qujianB(1))
			{
				MatrixXd tmp(1, 2);
				tmp(0) = qujianA(0);
				tmp(1) = qujianA(1);
				out = tmp;
			}
			else
			{
				MatrixXd tmp(1, 2);
				tmp(0) = qujianA(0);
				tmp(1) = qujianB(1);
				out = tmp;
			}
		}
	}
	return out;
}

MatrixXd GetIntersection(double qujianA, MatrixXd qujianB)
{
	MatrixXd A(1,1);
	A(0) = qujianA;
	MatrixXd out;
	out = GetIntersection(A, qujianB);
	return out;
}

MatrixXd GetIntersection(double qujianA, double qujianB)
{
	MatrixXd A(1, 1);
	A(0) = qujianA;
	MatrixXd B(1, 1);
	B(0) = qujianB;
	MatrixXd out;
	out = GetIntersection(A, B);
	return out;
}

MatrixXd GetIntersection(MatrixXd qujianA, double qujianB)
{
	MatrixXd B(1, 1);
	B(0) = qujianB;
	MatrixXd out;
	out = GetIntersection(qujianA, B);
	return out;
}

MatrixXd ones(int rows, int cols) //����rows��cols�е�1����
{
	if (rows < 0)
		error("ones����������Ϊ����");
	if (cols < 0)
		error("ones����������Ϊ����");

	MatrixXd result(rows,cols);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
			result(i, j) = 1;
	}

	return result;
}

MatrixXd zeros(int rows, int cols)
{
	if (rows < 0)
		error("ones����������Ϊ����");
	if (cols < 0)
		error("ones����������Ϊ����");

	MatrixXd result(rows, cols);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
			result(i, j) = 0;
	}

	return result;
}

int dot(MatrixXd vectorA, MatrixXd vectorB)
{
	if (vectorA.rows() != 1)
	{
		MatrixXd tmp;
		tmp = vectorA.transpose();
		vectorA = tmp;
	}
	if (vectorB.rows() != 1)
	{
		MatrixXd tmp;
		tmp = vectorB.transpose();
		vectorB = tmp;
	}
	if (vectorA.rows() != 1 || vectorB.rows() != 1)
	{
		error("dot������ʱ�޷������ά����");
	}
		
	if ((vectorA.rows() != vectorB.rows()) || (vectorA.cols() != vectorB.cols()))
	{
		error("��������ά����ͬ�����ܽ��е��");
	}

	int result = 0;
	for (int i = 0; i < vectorA.cols(); i++)
	{
		result += vectorA(i)*vectorB(i);
	}
	return result;
}

MatrixXd ExtractRows(MatrixXd mat, int rowsbegin, int rowsend)
{
	if (rowsend < rowsbegin)
		error("����λ��˳�򲻶�");
	if (rowsbegin < 0 || rowsbegin >= mat.rows() || rowsend < 0 || rowsend >= mat.rows())
		error("���󳬳�����");
	MatrixXd result(rowsend - rowsbegin + 1, mat.cols());
	int k = 0;
	for (int i = rowsbegin; i <= rowsend; i++)
	{
		result.row(k) = mat.row(i);
		k++;
	}
	return result;
}

MatrixXd ExtractCols(MatrixXd mat, int colsbegin, int colsend)
{
	if (colsend < colsbegin)
		error("����λ��˳�򲻶�");
	if (colsbegin < 0 || colsbegin >= mat.cols() || colsend < 0 || colsend >= mat.cols())
		error("���󳬳�����");
	MatrixXd result(mat.rows(),colsend - colsbegin + 1);
	int k = 0;
	for (int i = colsbegin; i <= colsend; i++)
	{
		result.col(k) = mat.col(i);
		k++;
	}
	return result;
}

MatrixXd GetRow_beginnum_Interval_endnum(double beginnum,double interval,double endnum)
{
	vector <double> tmpresult;

	for (double i = beginnum; i <= endnum; i = i + interval)
	{
		tmpresult.push_back(i);
	}
	MatrixXd result(1, tmpresult.size());
	for (int i = 0; i < tmpresult.size(); i++)
	{
		result(i) = tmpresult[i];
	}
	return result;
}
