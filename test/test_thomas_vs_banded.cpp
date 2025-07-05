#include <Eigen/Dense>

#include "solver/BlockThomasSolver.hpp"
#include "solver/BlockBandedSolver.hpp"

#include <iostream>
#include <chrono>

constexpr static int NumIters = 20;
constexpr static int BlockSize = 6;
constexpr static int NumBlocks = 100;
using ThomasSolverType = Solver::SymmetricBlockThomasSolver<BlockSize>;
using BandedSolverType = Solver::SymmetricBlockBandedSolver<BlockSize>;

ThomasSolverType::BlockMatType generateRandomSPDBlockMat()
{
    // fill matrix with random values
    typename ThomasSolverType::BlockMatType mat = ThomasSolverType::BlockMatType::Random();

    // make symmetric
    mat = 0.5*(mat*mat.transpose());

    // make diagonally dominant
    mat += ThomasSolverType::BlockSize * NumBlocks * ThomasSolverType::BlockMatType::Identity();

    // make sure that it is actually SPD
    Eigen::EigenSolver<ThomasSolverType::BlockMatType> solver(mat);
    Eigen::Vector<std::complex<Real>, BlockSize> eigenvalues = solver.eigenvalues();
    for (int i = 0; i < BlockSize; i++)
    {
        if (eigenvalues[i].imag() != Real(0.0) || eigenvalues[i].real() < 0)
            throw std::runtime_error("Non SPD matrix!");
    }

    Eigen::LLT<Eigen::MatrixXd> llt(mat);
    if (!mat.isApprox(mat.transpose()) || llt.info() == Eigen::NumericalIssue)
        throw std::runtime_error("Possibly non semi-positive definitie matrix!");

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
    // make diagonals

    // format for Thomas solver
    std::vector<ThomasSolverType::BlockMatType> A_diag_vec(NumBlocks);
    std::vector<ThomasSolverType::BlockMatType> A_off_diag_vec(NumBlocks-1);
    for (int i = 0; i < NumBlocks; i++)
        A_diag_vec[i] = generateRandomSPDBlockMat();
    for (int i = 0; i < NumBlocks-1; i++)
        A_off_diag_vec[i] = ThomasSolverType::BlockMatType::Random();

    // format for general block banded solver
    std::vector<std::vector<BandedSolverType::BlockMatType>> diagonals(2);
    diagonals[0].resize(NumBlocks, BandedSolverType::BlockMatType::Zero());
    diagonals[1].resize(NumBlocks, BandedSolverType::BlockMatType::Zero());
    for (int i = 0; i < NumBlocks; i++)
    {
        diagonals[0][i] = A_diag_vec[i];
    }
    for (int i = 0; i < NumBlocks-1; i++)
    {
        diagonals[1][i] = A_off_diag_vec[i];
    }
    MatXr full_mat = fullMatFromDiagonals(diagonals);

    std::cout << "\n=== THOMAS SOLVER ===" << std::endl;
    VecXr b = VecXr::Random(NumBlocks*BlockSize);
    VecXr x_thomas(NumBlocks*BlockSize);
    

    ThomasSolverType solver(NumBlocks);
    auto t_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NumIters; i++)
        solver.solve(A_diag_vec, A_off_diag_vec, b, x_thomas);
    auto t_end = std::chrono::high_resolution_clock::now();
    double ms_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count() / 1.0e6;
    std::cout << "Elapsed solve time (" <<  NumIters << " iterations): " << ms_elapsed << " ms" << std::endl;
    std::cout << "Average time per solve: " << ms_elapsed / NumIters << " ms" << std::endl;

    std::cout << "Solution residual: " << (full_mat * x_thomas - b).norm() << std::endl;


    // generic block banded solver
    std::cout << "\n=== BANDED SOLVER ===" << std::endl;

    
    VecXr x_banded(NumBlocks*BlockSize);
    BandedSolverType banded_solver(1, NumBlocks);

    t_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NumIters; i++)
        banded_solver.solve(diagonals, b, x_banded);
    t_end = std::chrono::high_resolution_clock::now();
    ms_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count() / 1.0e6;
    std::cout << "Elapsed solve time (" <<  NumIters << " iterations): " << ms_elapsed << " ms" << std::endl;
    std::cout << "Average time per solve: " << ms_elapsed / NumIters << " ms" << std::endl;

    std::cout << "Solution residual: " << (full_mat * x_banded - b).norm() << std::endl;

    // full LLT solver - also checks if full matrix is PSD
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



    // full matrix QR solve - works on non-PSD matrices
    std::cout << "\n=== FULL MATRIX HOUSEHOLDERQR SOLVER ===" << std::endl;
    VecXr x_householder(NumBlocks*BlockSize);
    
    t_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < NumIters; i++)
        x_householder = full_mat.colPivHouseholderQr().solve(b);
    t_end = std::chrono::high_resolution_clock::now();
    ms_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count() / 1.0e6;
    std::cout << "Elapsed solve time (" <<  NumIters << " iterations): " << ms_elapsed << " ms" << std::endl;
    std::cout << "Average time per solve: " << ms_elapsed / NumIters << " ms" << std::endl;

    std::cout << "Solution residual: " << (full_mat * x_householder - b).norm() << std::endl;
}