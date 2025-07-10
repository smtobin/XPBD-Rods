#ifndef __BLOCK_BANDED_SOLVER_HPP
#define __BLOCK_BANDED_SOLVER_HPP

#include "common/common.hpp"

#include <vector>

namespace Solver
{

/** Solves a linear system of equations that has a symmetric positive-definite, block-banded structure.
 * Block-banded refers to a matrix that has a small number of nonzero off-diagonals directly above and below the main the diagonal.
 * This structure can be exploited to solve the system of equations in O(np^3) time instead of O(n^3) time.
 * 
 * For example, a symmetric block matrix that has 3 total nonzero diagonals (the main diagonal, and the diagonals just above and below the main diagonals)
 *   is a familiar block-tridiagonal matrix.
 * For block-tridiagonal matrices, the bandwidth is 1, but we can also efficiently solve matrices with bandwidth > 1.
 * 
 * The block-banded algorithm is basically the same as a normal banded solver, and basically uses Gaussian elimination to transform a (n x n) block system of
 *   equations into a (n x n) block upper-triagonal system of equations which can be solved efficiently with back-substitution.
 * 
 * To use this solver, specify the block size (as a template parameter), the bandwidth, and the number of blocks on the main diagonal, and provide the diagonals
 * of the system matrix and the RHS vector. (diagonals[0] is the main diagonal, diagonals[1] is the first diagonal below the main, etc.)
 * 
 * For an example of how to use the solver, see test/test_banded.cpp or test/test_thomas_vs_banded.cpp.
 */
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

    /** Solves the system Ax=b, where A is symmetric and given in terms of its block diagonals.
     * Does not overwrite the input, but instead makes a copy.
     * 
     * @param input_diagonals : the diagonals of the system matrix.
     * input_diagonals[0] is taken as the main diagonal,
     * input_diagonals[1] is taken as the first diagonal below the main diagonal, etc.
     * Each diagonal should have size = the number of blocks on the main diagonal, and there should be
     * (bandwidth + 1) diagonals.
     * @param input_b : the RHS of the system
     * @param x : (OUTPUT) the solution to the system
     */
    void solve(const std::vector<std::vector<BlockMatType>>& input_diagonals, const VecXr& input_b, VecXr& x)
    {
        // make a copy of the diagonals and RHS vector and use solveInPlace
        std::vector<std::vector<BlockMatType>> diagonals = input_diagonals;
        VecXr b = input_b;
        solveInPlace(diagonals, b, x);
    }

    /** Solves the system Ax=b, where A is symmetric and given in terms of its block diagonals.
     * WILL OVERWRITE input diagonals and RHS vector, using them as scratch space for the algorithm.
     * 
     * @param diagonals : the original diagonals of the system matrix - will be overwritten
     * diagonals[0] is taken as the main diagonal,
     * diagonals[1] is taken as the first diagonal below the main diagonal, etc.
     * Each diagonal should have size = the number of blocks on the main diagonal, and there should be
     * (bandwidth + 1) diagonals.
     * @param b : the original RHS of the system - will be overwritten
     * @param x : (OUTPUT) the solution to the system
     */
    void solveInPlace(std::vector<std::vector<BlockMatType>>& diagonals, VecXr& b, VecXr& x)
    {
        // For each row:
        //  1) compute the modified RHS vector that goes in the RHS of the upper-triangular block system
        //  2) compute the modified diagonal blocks that go in the upper-triangular block system
        //  3) use the current row as a pivot to eliminate the row'th x variable from the remaining equations
        //      E.g. use row 1 (which looks like A_11 * x_1 + A_21^T * x_2 + ...) as a pivot, and subtract a multiple of row 1 from rows 2,3,etc. to
        //      eliminate x_1 from subsequent rows.
        for (int row = 0; row < _N; row++)
        {
            // compute Cholesky decomposition for main diagonal at current row
            _modified_diagonal[row] = diagonals[0][row].llt();


            // compute modified off-diagonals and RHS vector by multiplying the entire row through by H_ii^-T - these will go in upper-triangular system
            // 1) compute modified RHS vector for this row
            b.block<BlockSize,1>(BlockSize*row,0) = _modified_diagonal[row].matrixL().solve(b.block<BlockSize,1>(BlockSize*row,0));

            // 2) compute modified off-diagonals for this row
            for (int i = 0; i < _bandwidth; i++)
            {
                _modified_off_diagonals[i][row] = _modified_diagonal[row].matrixL().solve(diagonals[i+1][row].transpose());
            }

            // 3) use this row as pivot to eliminate leftmost x variable from the remaining equations
            // the number of equations that have the row'th x variable depends on the bandwidth
            for (int r = 1; r < _bandwidth+1; r++)
            {
                // if we reach the end of the matrix, stop
                if (row + r >= _N)
                    break;

                // compute the new blocks for the row (row+r) as a result of subtracting a multiple of the current row
                for (int i = 0; i < _bandwidth+1 - r; i++)
                {
                    diagonals[i][row+r] -= _modified_off_diagonals[r-1+i][row].transpose() * _modified_off_diagonals[r-1][row];
                }

                // compute the new RHS vec for the row (row+r) as a result of subtracting a multiple of the current row
                b.block<BlockSize,1>(BlockSize*(row+r),0) -= _modified_off_diagonals[r-1][row].transpose() * b.block<BlockSize,1>(BlockSize*row,0);
            }
        }


        // now we have a upper-triangular system, so we can solve easily by back-substitution
        // start at the end and work backwards
        for (int i = _N-1; i >= 0; i--)
        {
            // the RHS vector for this row
            BlockVecType rhs = b.block<BlockSize,1>(BlockSize*i,0);

            // subtract known parts of LHS from RHS so that we can solve for just the unknown part of x
            for (int j = 0; j < _bandwidth; j++)
            {
                if (j + i + 1 >= _N)
                    break;
                
                rhs -= _modified_off_diagonals[j][i] * x.block<BlockSize,1>(BlockSize*(i+j+1),0);
            }

            // solve for the unknown part of x
            x.block<BlockSize,1>(BlockSize*i,0) = _modified_diagonal[i].matrixU().solve(rhs);
        }
    }

    /** Sets the number of blocks on the main diagonal. */
    void setNumDiagBlocks(int num_diag_blocks)
    {
        _N = num_diag_blocks;

        // allocate space
        _modified_diagonal.resize(num_diag_blocks);
        for (int i = 0; i < _bandwidth; i++)
        {
            _modified_off_diagonals[i].resize(num_diag_blocks);
        }
    }

    /** Sets the bandwidth (i.e. number of nonzero off-diagonals). */
    void setBandwidth(int bandwidth)
    {
        _bandwidth = bandwidth;

        // allocate space
        _modified_off_diagonals.resize(_bandwidth);
        for (int i = 0; i < _bandwidth; i++)
        {
            _modified_off_diagonals[i].resize(_N);
        }

    }

    private:
    /** Expected bandwidth of the system matrix. */
    int _bandwidth;
    /** Expected number of blocks on the main diagonal. */
    int _N;

    /** Stores the the main diagonal of the upper triangular system, where each block of the diagonal is the upper-triangular Cholesky factor.
     * Storing the Cholesky decompositions directly should make it faster to solve systems of equations involving the Cholesky factor.
     */
    std::vector<Eigen::LLT<BlockMatType>> _modified_diagonal;

    /** Stores modified off-diagonals that form the reduced, upper triangular system.
     * First entry is the 1st off-diagonal, second entry is the 2nd off-diagonal, etc.
     */ 
    std::vector<std::vector<BlockMatType>> _modified_off_diagonals;

};

} // namespace Solver

#endif // __BLOCK_BANDED_SOLVER_HPP