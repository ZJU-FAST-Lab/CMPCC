#ifndef PROJECT_SPARSE_UTILS_H
#define PROJECT_SPARSE_UTILS_H

#include <Eigen/Sparse>
namespace sp{
    namespace rowMajor{
    inline void setBlock( // be careful to call this function!
        Eigen::SparseMatrix<double,Eigen::RowMajor> &big, 
        Eigen::SparseMatrix<double,Eigen::RowMajor> &small, //matrix small had better be really small
        int rowIndex, int colIndex)
        {
        for (int k=0; k<small.outerSize(); ++k){
            for (Eigen::SparseMatrix<double,Eigen::RowMajor>::InnerIterator it(small,k); it; ++it){
                big.coeffRef(rowIndex+it.row(),colIndex+it.col()) = it.value();
            }
        }
    };
    inline void setRows(
        Eigen::SparseMatrix<double,Eigen::RowMajor> &big, //n*n
        Eigen::SparseMatrix<double,Eigen::RowMajor> &small, //matrix small must be m*n, m<=n
        int rowIndex)
        {
        assert(big.cols()==small.cols());
        big.middleRows(rowIndex, small.rows()) = small;
    }
    inline void setCols(
        Eigen::SparseMatrix<double,Eigen::RowMajor> &big, 
        Eigen::SparseMatrix<double,Eigen::RowMajor> &small,
        int colIndex)
        {
        assert(big.rows() == small.rows());
        Eigen::SparseMatrix<double,Eigen::RowMajor> bigT = big.transpose();
        bigT.middleRows(colIndex, small.cols()) = small.transpose();
        big = bigT.transpose();
    } 
    inline void addCols(
        Eigen::SparseMatrix<double,Eigen::RowMajor> &big, 
        Eigen::SparseMatrix<double,Eigen::RowMajor> &small)
        {
        if (big.rows() == 0 && big.cols() == 0){
            big = small;
            return;
        };
        assert(big.rows() == small.rows());
        Eigen::SparseMatrix<double,Eigen::RowMajor> bigOld = big;
        big.resize(bigOld.rows(),bigOld.cols()+small.cols());
        setCols(big, bigOld, 0);
        setCols(big, small, bigOld.cols());
    }

    inline void addRows(
        Eigen::SparseMatrix<double,Eigen::RowMajor> &big, 
        Eigen::SparseMatrix<double,Eigen::RowMajor> &small)
        {
        if (big.rows() == 0 && big.cols() == 0){
            big = small;
            return;
        };
        assert(big.cols() == small.cols());
        Eigen::SparseMatrix<double,Eigen::RowMajor> bigOld(big);
        big.resize(bigOld.rows()+small.rows(), bigOld.cols());
        setRows(big, bigOld, 0);
        setRows(big, small, bigOld.rows());
    }

    inline void addBlock(
        Eigen::SparseMatrix<double,Eigen::RowMajor> &big, 
        Eigen::SparseMatrix<double,Eigen::RowMajor> &small)
        {
        if (big.rows() == 0 && big.cols() == 0){
            big = small;
            return;
        };
        Eigen::SparseMatrix<double,Eigen::RowMajor> up(big.rows(), big.cols()+small.cols());
        Eigen::SparseMatrix<double,Eigen::RowMajor> down(small.rows(), big.cols() + small.cols());
        setCols(up, big, 0);
        setCols(down, small, big.cols());
        addRows(up, down);
        big = up;
    }
    }// namespace rowMajor

    namespace colMajor{
    inline void setBlock( // be careful to call this function!
        Eigen::SparseMatrix<double> &big, 
        Eigen::SparseMatrix<double> &small, //matrix small had better be really small
        int rowIndex, int colIndex)
        {
        for (int k=0; k<small.outerSize(); ++k){
            for (Eigen::SparseMatrix<double>::InnerIterator it(small,k); it; ++it){
                big.coeffRef(rowIndex+it.row(),colIndex+it.col()) = it.value();
            }
        }
    };
    inline void setCols(
        Eigen::SparseMatrix<double> &big, //n*n
        Eigen::SparseMatrix<double> &small, //matrix small must be m*n, m<=n
        int colIndex)
        {
        assert(big.rows()==small.rows());
        big.middleCols(colIndex, small.cols()) = small;
    }
    inline void setRows(
        Eigen::SparseMatrix<double> &big, 
        Eigen::SparseMatrix<double> &small,
        int rowIndex)
        {
        assert(big.cols() == small.cols());
        Eigen::SparseMatrix<double> bigT = big.transpose();
        bigT.middleCols(rowIndex, small.rows()) = small.transpose();
        big = bigT.transpose();
    } 
    inline void addCols(
        Eigen::SparseMatrix<double> &big, 
        Eigen::SparseMatrix<double> &small)
        {
        if (big.rows() == 0 && big.cols() == 0){
            big = small;
            return;
        };
        assert(big.rows() == small.rows());
        Eigen::SparseMatrix<double> bigOld = big;
        big.resize(bigOld.rows(),bigOld.cols()+small.cols());
        setCols(big, bigOld, 0);
        setCols(big, small, bigOld.cols());
    }

    inline void addRows(
        Eigen::SparseMatrix<double> &big, 
        Eigen::SparseMatrix<double> &small)
        {
        if (big.rows() == 0 && big.cols() == 0){
            big = small;
            return;
        };
        assert(big.cols() == small.cols());
        Eigen::SparseMatrix<double> bigOld(big);
        big.resize(bigOld.rows()+small.rows(), bigOld.cols());
        setRows(big, bigOld, 0);
        setRows(big, small, bigOld.rows());
    }

    inline void addBlock(
        Eigen::SparseMatrix<double> &big, 
        Eigen::SparseMatrix<double> &small)
        {
        if (big.rows() == 0 && big.cols() == 0){
            big = small;
            return;
        };
        Eigen::SparseMatrix<double> left(big.rows()+small.rows(), big.cols());
        Eigen::SparseMatrix<double> right(big.rows()+small.rows(), small.cols());
        setRows(left, big, 0);
        setRows(right, small, big.rows());
        addCols(left, right);
        big = left;
    }
    }// namespace colMajor
}// namespace sp

#endif // PROJECT_SPARSE_UTILS_H
