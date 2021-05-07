#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_INTEGRATORS_WARD_H
#define PBRT_INTEGRATORS_WARD_H

// integrators/ward.h*
#include "pbrt.h"
#include "integrator.h"
#include "scene.h"

namespace pbrt {
class WardIntegrator : public SamplerIntegrator {
  public:
    //constructor
    WardIntegrator(std::shared_ptr<const Camera> camera,
		   std::shared_ptr<Sampler> sampler,
		   const Bounds2i &pixelBounds)
      : SamplerIntegrator(camera, sampler, pixelBounds) {}

    Spectrum Li(const RayDifferential &ray, const Scene &scene,
                Sampler &sampler, MemoryArena &arena, int depth) const;

    void Preprocess(const Scene &scene, Sampler &sampler);

  private:
    //represents a light source as a single point
    struct LightSource{

      const std::shared_ptr<Light> *light; //light source the point belongs too
      Point2f uLight; //position of the point on the source
      //potential contribution for the point being sampled
      //int nSamples; number of samples requested on the area light the source was sampled, may be implemented later to weight the contribution 
      int sampledCount; //number of times the source was sampled
      int sampleReached;//number of times the shadow ray reached the source successfully when sampling

      //TODO : complete once potential contribution is added
      /*bool operator<(const LightSource& lightSource){
	return 
	}*/
    };

    std::vector<LightSource>lightSources;  
};

WardIntegrator *CreateWardIntegrator(
    const ParamSet &params, std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera);


  
/*// LightStrategy Declarations
enum class LightStrategy { UniformSampleAll, UniformSampleOne };

// DirectLightingIntegrator Declarations
class DirectLightingIntegrator : public SamplerIntegrator {
  public:
    // DirectLightingIntegrator Public Methods
    DirectLightingIntegrator(LightStrategy strategy, int maxDepth,
                             std::shared_ptr<const Camera> camera,
                             std::shared_ptr<Sampler> sampler,
                             const Bounds2i &pixelBounds)
        : SamplerIntegrator(camera, sampler, pixelBounds),
          strategy(strategy),
          maxDepth(maxDepth) {}
    Spectrum Li(const RayDifferential &ray, const Scene &scene,
                Sampler &sampler, MemoryArena &arena, int depth) const;
    void Preprocess(const Scene &scene, Sampler &sampler);

  private:
    // DirectLightingIntegrator Private Data
    const LightStrategy strategy;
    const int maxDepth;
    std::vector<int> nLightSamples;
};

DirectLightingIntegrator *CreateDirectLightingIntegrator(
    const ParamSet &params, std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera);
*/

}  // namespace pbrt

#endif  // PBRT_INTEGRATORS_WARD_H
