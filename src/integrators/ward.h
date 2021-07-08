#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_INTEGRATORS_WARD_H
#define PBRT_INTEGRATORS_WARD_H

// integrators/ward.h*
#include <mutex>
//#include <atomic>
#include "pbrt.h"
#include "integrator.h"
#include "scene.h"

namespace pbrt {

//represents a light source as a single point
struct LightSource{  
  const std::shared_ptr<Light> *light; //light source the point belongs too
  Point2f uLight; //position of the point on the source
  int nSamples; //number of samples requested on the light source
  //std::atomic<int> nSampled{0};
  //std::atomic<int> nHits{0};
  int nSampled = 0; //number of times the source was sampled
  int nHits = 0; //number of times the source was hit
};


struct LocalSource{
  LightSource* lightSource;
  Spectrum ld = NULL; //store the contribution of the source for a given point
  VisibilityTester visibility;
  float luminance = 0.0f;
  bool updateSampled = false;
  bool updateHits =false;
  
  bool operator>(const LocalSource& rhs) const {
    return luminance > rhs.luminance;
  }
};

  
class WardIntegrator : public SamplerIntegratorBis {
  public:
    //constructor
    WardIntegrator(int maxDepth, std::shared_ptr<const Camera> camera,
		   std::shared_ptr<Sampler> sampler,
		   const Bounds2i &pixelBounds, float certainty, float tolerance)
      : SamplerIntegratorBis(camera, sampler, pixelBounds),
        certainty(certainty), maxDepth(maxDepth), tolerance(tolerance) {}

    void Preprocess(const Scene &scene, Sampler &sampler);

    Spectrum Li(const RayDifferential &ray,
              const Scene &scene, Sampler &sampler,
	      MemoryArena &arena, int depth);

  protected:
    void updateSourcesStats(std::vector<LocalSource> &localSources);
    //checks if the stopping criteria for evaluating source visibility is met after source number i in localSources
    bool isStopCriteriaMet(const std::vector<LocalSource> &localSources, const float sumLuminance, const int i);
  
    float certainty, tolerance ;
    const int maxDepth;
    std::vector<LightSource>lightSources;
    std::mutex mtx;

};

  WardIntegrator *CreateWardIntegrator(
    const ParamSet &params, std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera);


}  // namespace pbrt

#endif  // PBRT_INTEGRATORS_WARD_H
