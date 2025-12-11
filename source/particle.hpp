#pragma once

#include "constants.hpp"
#include "laser.hpp"
#include "occupancy.hpp"

#include <cstdint>
#include <random>
#include <span>
#include <vector>

namespace reanaut
{

const int kNumParticles = 500;

struct Particle : Pose
{
    Real weight{};
};

class ParticleFilter
{
public:

    using Real = RealType;

    static constexpr Real kYawTolerance  = 0.001;
    static constexpr Real kWeightEpsilon = 1e-9;

    ParticleFilter(uint32_t seed);

    void init(const Pose& pose);
    void prediction(Real velocity, Real yawRate, Real dt);
    void updateWeights(const std::vector<LaserScan>& scans, const OccupancyGrid& map);
    void resample();

    [[nodiscard]] auto getBestEstimate() const -> Particle;
    [[nodiscard]] auto particles() const -> std::span<const Particle> { return m_particles; }

protected:

    static auto gaussianProb(Real mu, Real sigma, Real value) -> Real;

private:

    std::vector<Particle>          m_particles;
    std::vector<Particle>          m_resampleBuffer;
    std::vector<Real>              m_weights;
    std::default_random_engine     m_gen;
    std::normal_distribution<Real> m_distX;
    std::normal_distribution<Real> m_distY;
    std::normal_distribution<Real> m_distTheta;
};

} // namespace reanaut
