// integrators/deterministDirect.cpp*
#include "integrators/deterministdirect.h"
#include "interaction.h"
#include "paramset.h"
#include "camera.h"
#include "film.h"
#include "stats.h"

namespace pbrt {
	
void DeterministDirectIntegrator::Preprocess(const Scene &scene,
                                Sampler &sampler) {
  std::vector<int> nLightSamples;
  //create a new instance of the sampler for sampling during preprocess
  std::unique_ptr<Sampler> preSampler = sampler.Clone(0);
  //initialize the sampler (the point used is not important in this case)
  
  for (const auto &light : scene.lights) {
    nLightSamples.push_back(preSampler->RoundCount(light->nSamples));
  }
  
  //Request samples for sampling all lights
  for (size_t i = 0; i < scene.lights.size(); ++i) {
    preSampler->Request2DArray(nLightSamples[i]);
  }
  Point2i pixel(0.0, 0.0);
  preSampler->StartPixel(pixel);
  preSampler->GetCameraSample(pixel);
  //sample points on the lights and put the resulting point lights in the lightSources vector to make re-ordering them easier later on
  for (size_t i = 0; i < scene.lights.size(); ++i) {
    int nSamples = nLightSamples[i];
    const Point2f *uLightArray = preSampler->Get2DArray(nSamples);
    //if the array wasn't sampled correctly, sample the source normally
    if(!uLightArray){
      //store a point on the source
      LightSource newSource {};
      newSource.light = &scene.lights[i];
      newSource.uLight = preSampler->Get2D();
      newSource.nSamples = 1;
      lightSources.push_back(newSource);
    }
    else{
      for (int j = 0; j < nSamples; ++j){
	//store a point on the source
	LightSource newSource {};
	newSource.light = &scene.lights[i];
	newSource.uLight = uLightArray[j];
	newSource.nSamples = nSamples;
	lightSources.push_back(newSource);
      }
    }
  }
}

Spectrum DeterministDirectIntegrator::Li(const RayDifferential &ray,
                                      const Scene &scene, Sampler &sampler,
                                      MemoryArena &arena, int depth) const {
  BxDFType bsdfFlags = BxDFType(BSDF_ALL & ~BSDF_SPECULAR);
  Spectrum L(0.f);
  SurfaceInteraction isect;
  //if no intersection found, consider that there is no incoming light (not considering environment maps)
  if (!scene.Intersect(ray, &isect)) {
    return L;
  }

  isect.ComputeScatteringFunctions(ray, arena);
  if (!isect.bsdf){
    return Li(isect.SpawnRay(ray.d), scene, sampler, arena, depth);
  }
  Vector3f wo = isect.wo;
  L += isect.Le(wo);
  if (lightSources.size() > 0) {
    for(int i = 0; i<lightSources.size();++i){
      Vector3f wi;
      Float lightPdf = 0;
      VisibilityTester visibility;
      Point2f uLight = sampler.Get2D();
      Spectrum Li = (*lightSources[i].light)->Sample_Li(isect, lightSources[i].uLight, &wi, &lightPdf, &visibility);
      if (lightPdf > 0 && !Li.IsBlack()) {
	//compute the BSDF
	Spectrum f;
	f = isect.bsdf->f(isect.wo, wi, bsdfFlags) * AbsDot(wi, isect.shading.n);
	if (!f.IsBlack()) {
	  if (!visibility.Unoccluded(scene)) {
	    Li = Spectrum(0.f);
	  }
	  
	  if (!Li.IsBlack()) {
	    L += (f * Li / lightPdf) / lightSources[i].nSamples;
	  }
	}
      }
    }
  }
  if (depth + 1 < maxDepth) {
    // Trace rays for specular reflection and refraction
    L += SpecularReflect(ray, isect, scene, sampler, arena, depth);
    L += SpecularTransmit(ray, isect, scene, sampler, arena, depth);
  }
  return L;
}
  
DeterministDirectIntegrator *CreateDeterministDirectIntegrator(
    const ParamSet &params, std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera) {
    int np;
    const int *pb = params.FindInt("pixelbounds", &np);
    int maxDepth = params.FindOneInt("maxdepth", 5);
    Bounds2i pixelBounds = camera->film->GetSampleBounds();
    if (pb) {
        if (np != 4)
            Error("Expected four values for \"pixelbounds\" parameter. Got %d.",
                  np);
        else {
            pixelBounds = Intersect(pixelBounds,
                                    Bounds2i{{pb[0], pb[2]}, {pb[1], pb[3]}});
            if (pixelBounds.Area() == 0)
                Error("Degenerate \"pixelbounds\" specified.");
        }
    }
    return new DeterministDirectIntegrator(maxDepth, camera, sampler, pixelBounds);
  }		
}  // namespace pbrt
