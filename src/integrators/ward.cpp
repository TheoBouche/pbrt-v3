// integrators/deterministDirect.cpp*
#include "integrators/ward.h"
#include "interaction.h"
#include "paramset.h"
#include "camera.h"
#include "film.h"
#include "stats.h"
#include <math.h>

namespace pbrt {

void WardIntegrator::Preprocess(const Scene &scene,
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

Spectrum WardIntegrator::Li(const RayDifferential &ray,
                                      const Scene &scene, Sampler &sampler,
                                      MemoryArena &arena, int depth) {
  BxDFType bsdfFlags = BxDFType(BSDF_ALL & ~BSDF_SPECULAR);
  Spectrum L(0.f);
  SurfaceInteraction isect;
  //if no intersection found, consider that there is no incoming light (not considering environment maps)
  if (!scene.Intersect(ray, &isect)) {
    for (const auto &light : scene.lights) L += light->Le(ray);
    return L;
  }
  
  isect.ComputeScatteringFunctions(ray, arena);
  if (!isect.bsdf){
    return Li(isect.SpawnRay(ray.d), scene, sampler, arena, depth);
  }
  Vector3f wo = isect.wo;
  L += isect.Le(wo);
  if (lightSources.size() > 0) {
    
    std::vector<LocalSource> localSources; //used to avoid modifying the integrator
    for(int i = 0; i < lightSources.size();++i){
      Vector3f wi;
      Float lightPdf = 0;
      VisibilityTester visibility;
      Point2f uLight = sampler.Get2D();
      Spectrum Li = (*lightSources[i].light)->Sample_Li(isect, lightSources[i].uLight, &wi, &lightPdf, &visibility);
      Spectrum ld(0.0f);
      if (lightPdf > 0 && !Li.IsBlack()) {
	//compute the BSDF
	Spectrum f;
	f = isect.bsdf->f(isect.wo, wi, bsdfFlags) * AbsDot(wi, isect.shading.n);
	if (!f.IsBlack()) {
	  ld = (f * Li / lightPdf) / lightSources[i].nSamples;
	}
      }
      LocalSource newLocalSource {};
      newLocalSource.lightSource = &lightSources[i];
      newLocalSource.ld = ld;
      newLocalSource.luminance = ld.y();
      newLocalSource.visibility = visibility;
      localSources.push_back (newLocalSource);
    }
    std::sort(localSources.begin(), localSources.end(),std::greater<LocalSource>()); //sorts in descending order   
    int sourcesSampled = 0; //number of light sources a ray was traced to from the point being evaluated
    int sourcesHit = 0; //number of tested sources visible from the point
    int i =0;
    float sumLuminance = 0.0f;
    //test source visibility until stopping criteria
    do {
      if(!localSources[i].ld.IsBlack()){
	localSources[i].updateSampled = true;
	//localSources[i].lightSource->nSampled.fetch_add(1);
	sourcesSampled++;
	if(localSources[i].visibility.Unoccluded(scene)) {
	  L+= localSources[i].ld;
	  localSources[i].updateHits = true;
	  //localSources[i].lightSource->nHits.fetch_add(1);
	  sourcesHit++;
	  sumLuminance += localSources[i].luminance;
	}
      }
      i++;
    } while(i < localSources.size() && !isStopCriteriaMet(localSources, sumLuminance, i));
    //update the number of times each source was sampled and hit
    updateSourcesStats (localSources);
    //approximate the remaining sources
    for(i; i < localSources.size(); ++i) {
      if(localSources[i].lightSource->nSampled != 0 && sourcesSampled != 0){
	L += localSources[i].ld * (sourcesHit / sourcesSampled) * (localSources[i].lightSource->nHits / localSources[i].lightSource->nSampled);
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

bool WardIntegrator::isStopCriteriaMet(const std::vector<LocalSource> &localSources, const float sumLuminance, const int i){
  int toCompareTo = std::round((localSources.size() - i) * certainty ) + i;//Number of sources past i to compare the sum to
  float sumLuminancePost = 0.0f; //sum of the luminance of the sources accounted for after source i
  for(int k = i; k < toCompareTo; ++k){
    sumLuminancePost += localSources[k].luminance;
  }
  return tolerance * sumLuminance >= sumLuminancePost;
}

void WardIntegrator::updateSourcesStats(std::vector<LocalSource> &localSources) {
  mtx.lock();
  for(int i = 0; i < localSources.size(); ++i) {
    if(localSources[i].updateSampled){
      localSources[i].lightSource->nSampled++;
    }
    if(localSources[i].updateHits){
      localSources[i].lightSource->nHits++;
    }
  }
  mtx.unlock();
}
  
WardIntegrator *CreateWardIntegrator(
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
    float certainty = params.FindOneFloat("certainty",1.0f);
    float tolerance = params.FindOneFloat("tolerance",0.1f);
    if(certainty > 1.0f || certainty < 0.0f || tolerance > 1.0f || tolerance < 0.0f) {
      Error ("Expected certainty and tolerance to be between 0 and 1");
    }
    return new WardIntegrator(maxDepth, camera, sampler, pixelBounds, certainty, tolerance);
  }
}  // namespace pbrt
