#ifndef EIGENMATRIXOPERATE_H
#define EIGENMATRIXOPERATE_H
#include "main.h"

//���һЩ�Զ���Ĳ���EigenMatrix������ĺ��� �������ǽ��MATLAB��

MatrixXd abs(MatrixXd mat);

MatrixXd find(MatrixXd Input, string Operator, double num);

MatrixXd addMatrix2Matrix(MatrixXd ResultMatrix, string Operator, MatrixXd addMatrix);

int isempty(MatrixXd mat);

#endif
