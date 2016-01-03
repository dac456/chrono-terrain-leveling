#include "ParticleSystem.hpp"
#include "Noise/PerlinNoise.h"

ParticleSystem::ParticleSystem(ChSystem* system, ChVectord dimensions, double density, double particleSize){
    PerlinNoise noiseFn;

    const double particleStep = particleSize*2.0;
    for(double i=-dimensions.x*0.5; i<dimensions.x*0.5; i+=particleStep){
        for(double j=-dimensions.x*0.5; j<dimensions.z*0.5; j+=particleStep){
            double height = noiseFn.noise(i, j, 0) * dimensions.y;

            for(double k=0.0; k<height; k+=particleStep){
                ChBodyPtr body = ChBodyPtr(new ChBody(DEFAULT_BODY));
                body->SetCollide(true);

                //ChVectord pos(-0.5 * dimensions.x + ChRandom() * dimensions.x, ChRandom() * dimensions.y, -0.5 * dimensions.z + ChRandom() * dimensions.z);
                ChVectord pos(i, k, j);
                ChFrameMoving<> bodyFrame(pos, QUNIT);
                body->ConcatenatePreTransformation(bodyFrame);

                body->GetCollisionModel()->ClearModel();
                body->GetCollisionModel()->AddSphere(particleSize);
                body->GetCollisionModel()->BuildModel();

                double mass = (4.0 / 3.0) * CH_C_PI * pow(particleSize, 3.0) * density;
                double inertia = pow(particleSize, 2) * mass;
                body->SetMass(mass);
                body->SetInertiaXX(ChVectord(inertia,inertia,inertia));
                body->GetMaterialSurface()->SetFriction(0.7);

                ChSharedPtr<ChSphereShape> shape(new ChSphereShape);
                shape->GetSphereGeometry().rad = particleSize;
                shape->GetSphereGeometry().center = ChVectord(0,0,0);

                body->AddAsset(shape);

                _particles.push_back(body);
                system->AddBody(body);
            }
        }
    }

}

ParticleSystem::~ParticleSystem(){
    _particles.clear();
}
