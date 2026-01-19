#pragma once

#include "common/common.hpp"

#include <iostream>
#include <vector>
#include <array>

namespace Solver
{

template<int MatSize_, int BlockSize_>
class FixedSizeSymmetricBlockThomasSolver
{
    public:
    static constexpr int MatSize = MatSize_;
    static constexpr int BlockSize = BlockSize_;
    static constexpr int NumBlocks = MatSize / BlockSize;

    using TotalMatType = Eigen::Matrix<double, MatSize, MatSize>;
    using TotalVecType = Eigen::Matrix<double, MatSize, 1>;
    using BlockMatType = Eigen::Matrix<double, BlockSize, BlockSize>;
    using BlockVecType = Eigen::Matrix<double, BlockSize, 1>;

    using DiagBlockArrayType = std::array<BlockMatType,NumBlocks>;
    using OffDiagBlockArrayType = std::array<BlockMatType, NumBlocks-1>;

    static_assert(MatSize % BlockSize == 0);

    static void solve(const DiagBlockArrayType& A_diag, const OffDiagBlockArrayType& A_off_diag, const TotalVecType& b, TotalVecType& x)
    {
        std::array<Eigen::LLT<BlockMatType>, NumBlocks> H_ii;   // stores diagonal modified blocks
        OffDiagBlockArrayType H_iplus1_i;       // stores off-diagonal modified blocks
        std::array<BlockVecType, NumBlocks> c_i;                // stores modified RHS
        
        // initial iterates H_11 and c_1
        H_ii[0] = A_diag[0].llt();
        c_i[0] = H_ii[0].matrixL().solve(b.template block<BlockSize, 1>(0,0));

        // compute the rest of H_ii and H_(i+1),i
        for (int i = 1; i < NumBlocks; i++)
        {
            H_iplus1_i[i-1] = H_ii[i-1].matrixL().solve(A_off_diag[i-1].transpose());

            const BlockMatType A_ii_new = A_diag[i] - H_iplus1_i[i-1].transpose() * H_iplus1_i[i-1];
            H_ii[i] = A_ii_new.llt();
            c_i[i] = H_ii[i].matrixL().solve( b.template block<BlockSize, 1>(BlockSize*i, 0) - H_iplus1_i[i-1].transpose() * c_i[i-1]);
        }

        // back substitution to solve for x
        x.template block<BlockSize, 1>(BlockSize*(NumBlocks-1), 0) = H_ii[NumBlocks-1].matrixU().solve(c_i[NumBlocks-1]);
        for (int i = NumBlocks-2; i >= 0; i--)
        {
            x.template block<BlockSize, 1>(BlockSize*i, 0) = 
                H_ii[i].matrixU().solve( c_i[i] - H_iplus1_i[i] * x.template block<BlockSize, 1>(BlockSize*(i+1), 0) );
        }
    }

};

template<int BlockSize_>
class SymmetricBlockThomasSolver
{
    public:
    static constexpr int BlockSize = BlockSize_;

    using BlockMatType = Eigen::Matrix<Real, BlockSize, BlockSize>;
    using BlockVecType = Eigen::Matrix<Real, BlockSize, 1>;

    SymmetricBlockThomasSolver(const int num_diag_blocks)
        : _H_ii(num_diag_blocks), _H_iplus1_i(num_diag_blocks-1), _c_i(num_diag_blocks)
    {

    }

    void solve(const std::vector<BlockMatType>& A_diag, const std::vector<BlockMatType>& A_off_diag, const VecXr& b, VecXr& x)
    {
        // make sure the sizes of everything are correct
        assert(A_diag.size() == A_off_diag.size()+1);
        assert(A_diag.size() * BlockSize == static_cast<unsigned long>(b.size()));
        assert(A_diag.size() == _H_ii.size());           
        
        // initial iterates H_11 and c_1
        _H_ii[0] = A_diag[0].llt();
        _c_i[0] = _H_ii[0].matrixL().solve(b.template block<BlockSize, 1>(0,0));

        // compute the rest of H_ii and H_(i+1),i
        for (unsigned i = 1; i < A_diag.size(); i++)
        {
            _H_iplus1_i[i-1] = _H_ii[i-1].matrixL().solve(A_off_diag[i-1].transpose());

            const BlockMatType A_ii_new = A_diag[i] - _H_iplus1_i[i-1].transpose() * _H_iplus1_i[i-1];
            _H_ii[i] = A_ii_new.llt();
            _c_i[i] = _H_ii[i].matrixL().solve( b.template block<BlockSize, 1>(BlockSize*i, 0) - _H_iplus1_i[i-1].transpose() * _c_i[i-1]);
        }

        // back substitution to solve for x
        x.template block<BlockSize, 1>(BlockSize*(A_diag.size()-1), 0) = _H_ii[A_diag.size()-1].matrixU().solve(_c_i[A_diag.size()-1]);
        for (int i = A_diag.size()-2; i >= 0; i--)
        {
            x.template block<BlockSize, 1>(BlockSize*i, 0) = 
                _H_ii[i].matrixU().solve( _c_i[i] - _H_iplus1_i[i] * x.template block<BlockSize, 1>(BlockSize*(i+1), 0) );
        }
    }

    void setNumDiagBlocks(int num_diag_blocks)
    {
        _H_ii.resize(num_diag_blocks);
        _H_iplus1_i.resize(num_diag_blocks-1);
        _c_i.resize(num_diag_blocks);
    }

    private:
    std::vector<Eigen::LLT<BlockMatType>> _H_ii;    // stores diagonal modified blocks
    std::vector<BlockMatType> _H_iplus1_i;          // stores off-diagonal modified blocks
    std::vector<BlockVecType> _c_i;                 // stores modified RHS

};

} // namespace Solver