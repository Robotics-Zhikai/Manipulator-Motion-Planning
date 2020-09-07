#ifndef EIGENMATRIXOPERATE_H
#define EIGENMATRIXOPERATE_H
#include "main.h"

//存放一些自定义的操作EigenMatrix函数库的函数 大体上是借鉴MATLAB的

MatrixXd abs(MatrixXd mat);

MatrixXd find(MatrixXd Input, string Operator, double num);

MatrixXd addMatrix2Matrix(MatrixXd ResultMatrix, string Operator, MatrixXd addMatrix);

int isempty(MatrixXd mat);

#endif
