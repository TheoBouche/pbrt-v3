#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_INTEGRATORS_DETERMINISTDIRECT_H
#define PBRT_INTEGRATORS_DETERMINISTDIRECT_H

// integrators/ward.h*
#include "pbrt.h"
#include "integrator.h"
#include "scene.h"

namespace pbrt {
class DeterministDirectIntegrator : public SamplerIntegrator {
  public:
    //constructor
    DeterministDirectIntegrator(int maxDepth, std::shared_ptr<const Camera> camera,
		   std::shared_ptr<Sampler> sampler,
		   const Bounds2i &pixelBounds)
      : SamplerIntegrator(camera, sampler, pixelBounds),
        maxDepth(maxDepth){}

    Spectrum Li(const RayDifferential &ray, const Scene &scene,
                Sampler &sampler, MemoryArena &arena, int depth) const;

    void Preprocess(const Scene &scene, Sampler &sampler);

  private:
    //represents a light source as a single point
    struct LightSource{

      const std::shared_ptr<Light> *light; //light source the point belongs too
      Point2f uLight; //position of the point on the source
      int nSamples; //number of samples requested on the light source
    };

    const int maxDepth;
    std::vector<LightSource>lightSources;
};

  DeterministDirectIntegrator *CreateDeterministDirectIntegrator(
    const ParamSet &params, std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera);

}  // namespace pbrt

#endif  // PBRT_INTEGRATORS_WARD_H
