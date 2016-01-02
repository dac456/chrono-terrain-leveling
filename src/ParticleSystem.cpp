#include "ParticleSystem.hpp"

ParticleSystem::ParticleSystem(ChSystem* system, ChVectord dimensions, double density, double particleSize){
    for(size_t i=0; i<4000; i++){
        ChBodyPtr body = ChBodyPtr(new ChBody(DEFAULT_BODY));
        body->SetCollide(true);

        ChVectord pos(-0.5 * dimensions.x + ChRandom() * dimensions.x, ChRandom() * dimensions.y, -0.5 * dimensions.z + ChRandom() * dimensions.z);
        ChFrameMoving<> bodyFrame(pos, QUNIT);
        body->ConcatenatePreTransformation(bodyFrame);

        body->GetCollisionModel()->ClearModel();
        body->GetCollisionModel()->AddSphere(particleSize);
        body->GetCollisionModel()->BuildModel();

        ChSharedPtr<ChSphereShape> shape(new ChSphereShape);
        shape->GetSphereGeometry().rad = particleSize;
        shape->GetSphereGeometry().center = ChVectord(0,0,0);

        body->AddAsset(shape);

        _particles.push_back(body);
        system->AddBody(body);
    }

}

ParticleSystem::~ParticleSystem(){
    _particles.clear();
}
