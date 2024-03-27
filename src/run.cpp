#include <iostream>
#include <chrono>
#include "straight_line.hpp"
#include "cornering.hpp"

int main()
{
    auto t1 = std::chrono::high_resolution_clock::now();
    int mode = 2; // 1 == accel run, 2 == SkidPad, 3 == TrackRuns

    // Initialize variables
    float timer = 0.0;
    float StateOfCharge = 20.0; // Ah
    float velocity = 0.0;

    switch (mode)
    {
    case 1:
    {
        float finalDistance = 75.0;
        straightLine(finalDistance, timer, StateOfCharge, velocity);
        break;
    }
    case 2:
    {
        // Track parameters
        float Turn1Radius = 9.125;
        float SectorLenght = 2 * M_PI * Turn1Radius;
        cornering(Turn1Radius, SectorLenght, timer, StateOfCharge, velocity);
        break;
    }
    default:
        break;
    }

    std::cout << "Lap time: " << timer << " seconds" << std::endl;
    std::cout << "Lap time: " << timer / 60 << " minutes" << std::endl;
    std::cout << "Accumulator Capacity: " << StateOfCharge * accVoltage << std::endl;

    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    std::cout << "Program Run time: " << duration / 1000 << " milliseconds" << std::endl; // milliseconds

    return 0;
}
