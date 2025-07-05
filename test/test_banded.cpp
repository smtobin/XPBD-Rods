#include <Eigen/Dense>

#include "solver/BlockBandedSolver.hpp"

#include <iostream>
#include <chrono>

constexpr static int NumIters = 20;
constexpr static int BlockSize = 6;
constexpr static int NumBlocks = 100;
constexpr static int Bandwidth = 2;
using BandedSolverType = Solver::SymmetricBlockBandedSolver<BlockSize>;

BandedSolverType::BlockMatType generateRandomSPDBlockMat()
{
    // fill matrix with random values
    typename BandedSolverType::BlockMatType mat = BandedSolverType::BlockMatType::Random();

    // make symmetric
    mat = 0.5*(mat*mat.transpose());

    // make diagonally dominant
    mat += BandedSolverType::BlockSize * NumBlocks * BandedSolverType::BlockMatType::Identity();

    return mat;
}

MatXr fullMatFromDiagonals(std::vector<std::vector<BandedSolverType::BlockMatType>> diagonals)
{
    MatXr mat(NumBlocks*BlockSize, NumBlocks*BlockSize);
    for (unsigned i = 0; i < diagonals.size(); i++)
    {
        for (int row = 0; row < NumBlocks-static_cast<int>(i); row++)
        {
            if (i == 0) // main diagonal
            {
                mat.block<BlockSize,BlockSize>(row*BlockSize, row*BlockSize) = diagonals[i][row];
            }
            else // off diagonal
            {
                mat.block<BlockSize,BlockSize>( (row+i)*BlockSize, row*BlockSize) = diagonals[i][row];
                mat.block<BlockSize,BlockSize>( row*BlockSize, (row+i)*BlockSize) = diagonals[i][row].transpose();
            }
        }
    }

    return mat;
}

int main() 
{
    
    // create diagonals of matrix according to bandwidth
    std::vector<std::vector<BandedSolverType::BlockMatType>> diagonals(Bandwidth+1);
    for (int b = 0; b < Bandwidth+1; b++)
    {
        diagonals[b].resize(NumBlocks, BandedSolverType::BlockMatType::Zero());
        for (int row = 0; row < NumBlocks-b; row++)
        {
            if (b == 0) // main diagonal
            {
                diagonals[b][row] = generateRandomSPDBlockMat();
            }
            else // if (b == 1) // off diagonal
            {
                diagonals[b][row] = BandedSolverType::BlockMatType::Random();
            }
        }
    }
    VecXr b = VecXr::Random(NumBlocks*BlockSize);
    MatXr full_mat = fullMatFromDiagonals(diagonals);

    // test banded solver
    std::cout << "=== BANDED SOLVER ===" << std::endl;
    

    VecXr x_banded(NumBlocks*BlockSize);
    BandedSolverType banded_solver(Bandwidth, NumBlocks);

    auto t_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NumIters; i++)
        banded_solver.solve(diagonals, b, x_banded);
    auto t_end = std::chrono::high_resolution_clock::now();
    double ms_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count() / 1.0e6;
    std::cout << "Elapsed solve time (" <<  NumIters << " iterations): " << ms_elapsed << " ms" << std::endl;
    std::cout << "Average time per solve: " << ms_elapsed / NumIters << " ms" << std::endl;

    std::cout << "Solution residual: " << (full_mat * x_banded - b).norm() << std::endl;

    std::cout << "\n=== FULL MATRIX LLT SOLVER ===" << std::endl;


    VecXr x_llt(NumBlocks*BlockSize);
    t_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NumIters; i++)
    {
        Eigen::LLT<Eigen::MatrixXd> full_llt(full_mat);
        if (!full_mat.isApprox(full_mat.transpose()) || full_llt.info() == Eigen::NumericalIssue)
            throw std::runtime_error("Possibly non semi-positive definite full matrix!");

        x_llt = full_llt.solve(b);
    }
    t_end = std::chrono::high_resolution_clock::now();
    ms_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count() / 1.0e6;
    std::cout << "Elapsed solve time (" <<  NumIters << " iterations): " << ms_elapsed << " ms" << std::endl;
    std::cout << "Average time per solve: " << ms_elapsed / NumIters << " ms" << std::endl;

    std::cout << "Solution residual: " << (full_mat * x_llt - b).norm() << std::endl;
}