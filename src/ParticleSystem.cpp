#include "ParticleSystem.hpp"
#include "Noise/PerlinNoise.h"

ParticleSystem::ParticleSystem(ChSystem* system, ChVectord dimensions, double particleDensity, double particleSize){
    ChSharedPtr<ChParticlesClones> chParticles(new ChParticlesClones);
    chParticles->SetCollide(true);

    chParticles->GetCollisionModel()->ClearModel();
    chParticles->GetCollisionModel()->AddSphere(particleSize);
    chParticles->GetCollisionModel()->BuildModel();

    double mass = (4.0 / 3.0) * CH_C_PI * pow(particleSize, 3.0) * particleDensity;
    double inertia = pow(particleSize, 2) * mass;
    chParticles->SetMass(mass);
    chParticles->SetInertiaXX(ChVectord(inertia,inertia,inertia));

    ChMaterialPtr particleMat(new ChMaterialSurface);
    particleMat->SetFriction(0.7);
    chParticles->SetMaterialSurface(particleMat);

    ChSharedPtr<ChSphereShape> shape(new ChSphereShape);
    shape->GetSphereGeometry().rad = particleSize;

    chParticles->AddAsset(shape);

    PerlinNoise noiseFn;

    const double particleStep = particleSize*2.0;
    for(double i=-dimensions.x*0.5; i<dimensions.x*0.5; i+=particleStep){
        for(double j=-dimensions.x*0.5; j<dimensions.z*0.5; j+=particleStep){
            double height = noiseFn.noise(i, j, 0) * dimensions.y;

            for(double k=0.0; k<height; k+=particleStep){
                //ChVectord pos(-0.5 * dimensions.x + ChRandom() * dimensions.x, ChRandom() * dimensions.y, -0.5 * dimensions.z + ChRandom() * dimensions.z);
                ChVectord pos(i, k, j);
                ChFrameMoving<> bodyFrame(pos, QUNIT);

                chParticles->AddParticle(bodyFrame.GetCoord());
            }
        }
    }

    system->Add(chParticles);

}

ParticleSystem::~ParticleSystem(){
    _particles.clear();
}
