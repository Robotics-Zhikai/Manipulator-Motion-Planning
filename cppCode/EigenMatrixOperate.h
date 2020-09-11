#ifndef EIGENMATRIXOPERATE_H
#define EIGENMATRIXOPERATE_H
#include "main.h"

//存放一些自定义的操作EigenMatrix函数库的函数 大体上是借鉴MATLAB的

MatrixXd abs(MatrixXd mat);

MatrixXd find(MatrixXd Input, string Operator, double num);

MatrixXd addMatrix2Matrix(MatrixXd ResultMatrix, string Operator, MatrixXd addMatrix);
MatrixXd addMatrix2Matrix(double num, string Operator, MatrixXd addMatrix);
MatrixXd addMatrix2Matrix(double num, string Operator, double addnum);
MatrixXd addMatrix2Matrix(MatrixXd num, string Operator, double addMatrix);

int isempty(MatrixXd mat);

int size(MatrixXd mat, int num);

MatrixXd GetIntersection(MatrixXd qujianA, MatrixXd qujianB);
MatrixXd GetIntersection(double qujianA, MatrixXd qujianB);
MatrixXd GetIntersection(double qujianA, double qujianB);
MatrixXd GetIntersection(MatrixXd qujianA, double qujianB);

MatrixXd ones(int rows, int cols); //生成rows行cols列的1矩阵
MatrixXd zeros(int rows, int cols);

int dot(MatrixXd vectorA, MatrixXd vectorB);

MatrixXd ExtractRows(MatrixXd mat, int rowsbegin, int rowsend);
MatrixXd ExtractCols(MatrixXd mat, int colsbegin, int colsend);
#endif
