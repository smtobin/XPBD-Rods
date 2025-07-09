#ifndef __BLOCK_BANDED_SOLVER_HPP
#define __BLOCK_BANDED_SOLVER_HPP

#include "common/common.hpp"

#include <vector>

namespace Solver
{

template<int BlockSize_>
class SymmetricBlockBandedSolver
{

    public:
    static constexpr int BlockSize = BlockSize_;

    using BlockMatType = Eigen::Matrix<Real, BlockSize, BlockSize>;
    using BlockVecType = Eigen::Matrix<Real, BlockSize, 1>;

    SymmetricBlockBandedSolver(int bandwidth, int num_diag_blocks)
        : _bandwidth(bandwidth), _N(num_diag_blocks)
    {
        _modified_diagonal.resize(num_diag_blocks);
        _modified_off_diagonals.resize(_bandwidth);
        for (int i = 0; i < _bandwidth; i++)
        {
            _modified_off_diagonals[i].resize(num_diag_blocks, BlockMatType::Zero());
        }

    }

    void solve(const std::vector<std::vector<BlockMatType>>& input_diagonals, const VecXr& input_b, VecXr& x)
    {
        std::vector<std::vector<BlockMatType>> diagonals = input_diagonals;
        VecXr b = input_b;
        solveInPlace(diagonals, b, x);
    }

    void solveInPlace(std::vector<std::vector<BlockMatType>>& diagonals, VecXr& b, VecXr& x)
    {
        for (int i = 0; i < _N; i++)
        {
            _reduceSubsystem(i, diagonals, b);
        }

        // once we have all the modified diagonals, we have a block upper triangular system so we can do backward substitution to solve
        for (int i = _N-1; i >= 0; i--)
        {
            BlockVecType rhs = b.block<BlockSize,1>(BlockSize*i,0);

            for (int j = 0; j < _bandwidth; j++)
            {
                if (j + i + 1 >= _N)
                    break;
                
                rhs -= _modified_off_diagonals[j][i] * x.block<BlockSize,1>(BlockSize*(i+j+1),0);
            }

            x.block<BlockSize,1>(BlockSize*i,0) = _modified_diagonal[i].matrixU().solve(rhs);
        }
    }

    void setNumDiagBlocks(int num_diag_blocks)
    {
        _N = num_diag_blocks;

        _modified_diagonal.resize(num_diag_blocks);
        for (int i = 0; i < _bandwidth; i++)
        {
            _modified_off_diagonals[i].resize(num_diag_blocks);
        }
    }

    void setBandwidth(int bandwidth)
    {
        _bandwidth = bandwidth;
        _modified_off_diagonals.resize(_bandwidth);
        for (int i = 0; i < _bandwidth; i++)
        {
            _modified_off_diagonals[i].resize(_N);
        }

    }

    private:
    // computes modified diagonals that form the reduced, upper triangular system
    void _reduceSubsystem(int row, std::vector<std::vector<BlockMatType>>& diagonals, VecXr& b)
    {
        // compute Cholesky decomposition for main diagonal at current row
        _modified_diagonal[row] = diagonals[0][row].llt();
        b.block<BlockSize,1>(BlockSize*row,0) = _modified_diagonal[row].matrixL().solve(b.block<BlockSize,1>(BlockSize*row,0));

        // compute modified off-diagonals
        for (int i = 0; i < _bandwidth; i++)
        {
            _modified_off_diagonals[i][row] = _modified_diagonal[row].matrixL().solve(diagonals[i+1][row].transpose());
        }

        // compute modified next rows
        for (int r = 1; r < _bandwidth+1; r++)
        {
            if (row + r >= _N)
                break;

            for (int i = 0; i < _bandwidth+1 - r; i++)
            {
                diagonals[i][row+r] -= _modified_off_diagonals[r-1+i][row].transpose() * _modified_off_diagonals[r-1][row];
            }

            b.block<BlockSize,1>(BlockSize*(row+r),0) -= _modified_off_diagonals[r-1][row].transpose() * b.block<BlockSize,1>(BlockSize*row,0);
        }
    }

    private:
    int _bandwidth;
    int _N;

    // stores the Cholesky-decomped modified main diagonal
    std::vector<Eigen::LLT<BlockMatType>> _modified_diagonal;

    // stores modified off-diagonals that form the reduced, upper triangular system
    // first entry is the 1st off-diagonal, second entry is the 2nd off-diagonal, etc.
    std::vector<std::vector<BlockMatType>> _modified_off_diagonals;

};

} // namespace Solver

#endif // __BLOCK_BANDED_SOLVER_HPP