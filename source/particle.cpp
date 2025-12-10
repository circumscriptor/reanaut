#include "constants.hpp"
#include "laser.hpp"
#include "occupancy.hpp"
#include "particle.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>
#include <vector>

namespace reanaut
{

ParticleFilter::ParticleFilter(uint32_t seed) : m_gen(seed), m_distX(0, kStdX), m_distY(0, kStdY), m_distTheta(0, kStdTheta)
{
    m_particles.resize(kNumParticles);
}

void ParticleFilter::init(const Pose& pose)
{
    for (int i = 0; i < kNumParticles; ++i) {
        m_particles[i] = {pose, 1.0};
    }
}

void ParticleFilter::prediction(double velocity, double yawRate, double dt)
{
    for (auto& particle : m_particles) {
        if (std::fabs(yawRate) < kYawTolerance) {
            particle.x += velocity * std::cos(particle.theta) * dt;
            particle.y += velocity * std::sin(particle.theta) * dt;
        } else {
            particle.x += (velocity / yawRate) * (std::sin(particle.theta + (yawRate * dt)) - std::sin(particle.theta));
            particle.y += (velocity / yawRate) * (std::cos(particle.theta) - std::cos(particle.theta + (yawRate * dt)));
            particle.theta += yawRate * dt;
        }
        // Add Process Noise
        particle.x += m_distX(m_gen);
        particle.y += m_distY(m_gen);
        particle.theta = std::fmod(particle.theta + m_distTheta(m_gen), std::numbers::pi);
    }
}

void ParticleFilter::updateWeights(const std::vector<LaserScan>& scans, const OccupancyGrid& map)
{
    double maxWeight = 0.0;

    for (auto& particle : m_particles) {
        double weight = 1.0;

        for (const auto& scan : scans) {
            const double obs = scan.distanceMm * 0.001;
            if (obs > kLidarMaxRange || obs < kLidarMinRange) {
                continue;
            }

            const double beamAngle  = scan.angleDeg * std::numbers::pi / 180.0;
            const double worldAngle = std::fmod(particle.theta + beamAngle, std::numbers::pi);

            // Raycast against the Occupancy Grid
            const double pred = map.getDistance({
                {.x = particle.x, .y = particle.y},
                worldAngle
            });

            // Weight based on difference
            weight *= gaussianProb(pred, kStdLidar, obs);
        }
        particle.weight = weight;
        maxWeight       = std::max(weight, maxWeight);
    }

    // Normalize
    maxWeight = std::max(maxWeight, kWeightEpsilon);
    for (auto& particle : m_particles) {
        particle.weight /= maxWeight;
    }
}

void ParticleFilter::resample()
{
    // Generate weight vector for distribution

    m_weights.clear();
    m_weights.reserve(kNumParticles);
    for (const auto& particle : m_particles) {
        m_weights.push_back(particle.weight);
    }

    std::discrete_distribution<int> dist(m_weights.begin(), m_weights.end());

    // Fill the buffer
    for (int i = 0; i < kNumParticles; ++i) {
        m_resampleBuffer[i]        = m_particles[dist(m_gen)];
        m_resampleBuffer[i].weight = 1.0; // Reset weight
    }

    // Swap buffer with main particle vector (avoids reallocation)
    std::swap(m_particles, m_resampleBuffer);
}

auto ParticleFilter::getBestEstimate() -> Particle
{
    double pX     = 0;
    double pY     = 0;
    double cosSum = 0;
    double sinSum = 0;

    for (const auto& particle : m_particles) {
        pX += particle.x;
        pY += particle.y;
        cosSum += std::cos(particle.theta);
        sinSum += std::sin(particle.theta);
    }

    return {
        {{.x = pX / kNumParticles, .y = pY / kNumParticles}, std::atan2(sinSum, cosSum)},
        0.0
    };
}

auto ParticleFilter::gaussianProb(Real mu, Real sigma, Real value) -> Real
{
    const Real expExponent = -0.5 * std::pow((value - mu) / sigma, 2);
    return (1.0 / (sigma * std::sqrt(2.0 * std::numbers::pi))) * exp(expExponent);
}

} // namespace reanaut
