#include "cloud.hpp"
#include "constants.hpp"
#include "laser.hpp"
#include "occupancy.hpp"
#include "particle.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
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

void ParticleFilter::prediction(Real velocity, Real yawRate, Real dt)
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
        particle.x += m_distX(m_gen) * dt;
        particle.y += m_distY(m_gen) * dt;
        particle.theta += m_distTheta(m_gen) * dt;
        particle.theta = std::atan2(std::sin(particle.theta), std::cos(particle.theta));
    }
}

void ParticleFilter::updateFromScans(const std::vector<LaserScan>& scans, const OccupancyGrid& map)
{
    Real maxWeight = 0.0;

    for (auto& particle : m_particles) {
        Real weight = 1.0;

        for (const auto& scan : scans) {
            const Real range = scan.toMeters();
            if (range > kLidarMaxRange || range < kLidarMinRange) {
                continue;
            }

            const Real worldAngle = scan.toWorldAngle(particle.theta);

            // Raycast against the Occupancy Grid
            const Real pred = map.getDistance({
                {.x = particle.x, .y = particle.y},
                worldAngle
            });

            // Weight based on difference
            weight *= gaussianProb(pred, kStdLidar, range);
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

void ParticleFilter::updateFromCloud(const PointCloud& cloud, const OccupancyGrid& map)
{
    Real maxWeight = 0.0;

    // Pre-calculate constants
    // If your map resolution is 5cm, sigma should be around 0.1m - 0.2m
    // const Real sigma = 0.15;

    for (auto& particle : m_particles) {
        Real weight = 1.0;

        const Real sinT = std::sin(particle.theta);
        const Real cosT = std::cos(particle.theta);

        for (const auto& point : cloud.points()) {
            // 1. Transform Point to World Space
            const Point2 pw{
                .x = particle.x + (point.x * cosT - point.y * sinT),
                .y = particle.y + (point.x * sinT + point.y * cosT),
            };

            // 2. Get Score from Map
            const auto score = map.getProbabilitySmooth(pw);
            if (not score) {
                continue;
            }

            // 3. Accumulate Weight
            weight *= *score;

            // A more mathematically rigorous Endpoint Model approach:
            // weight *= (score * gaussian_term + random_term);
            // But direct multiplication works fine for simple grids.
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

    Real sum{};
    for (const auto& particle : m_particles) {
        m_weights.push_back(particle.weight);
        sum += particle.weight;
    }

    // If sum is 0 (or very close), the filter is lost.
    if (sum <= std::numeric_limits<double>::epsilon()) {
        // Recovery: Force uniform weights.
        std::fill(m_weights.begin(), m_weights.end(), 1.0);
    }

    std::discrete_distribution<int> dist(m_weights.begin(), m_weights.end());

    // Fill the buffer
    m_resampleBuffer.clear();
    m_resampleBuffer.resize(m_particles.size());
    for (auto& particle : m_resampleBuffer) {
        particle        = m_particles[dist(m_gen)];
        particle.weight = 1.0; // Reset weight
    }

    // Swap buffer with main particle vector (avoids reallocation)
    std::swap(m_particles, m_resampleBuffer);
}

auto ParticleFilter::getBestEstimate() const -> Particle
{
    Real pX     = 0;
    Real pY     = 0;
    Real cosSum = 0;
    Real sinSum = 0;

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
