#include "ParticleSystem.hpp"
#include "HeightMap.hpp"
#include "Noise/PerlinNoise.h"

ParticleSystem::ParticleSystem(ChSystem* system, HeightMapPtr heightMap, double maxHeight, double particleDensity, double particleSize, bool container, bool visContainer){
    const double particleStep = particleSize*2.0;

    if(container){
        ChBodyPtr floor(new ChBody(DEFAULT_BODY));
        floor->SetCollide(true);
        floor->SetBodyFixed(true);
        floor->GetMaterialSurface()->SetFriction(1.0);

        ChFrameMoving<> floorFrame(ChVectord(0, -0.05 - particleSize, 0), QUNIT);
        floor->ConcatenatePreTransformation(floorFrame);

        floor->GetCollisionModel()->ClearModel();
        floor->GetCollisionModel()->AddBox((heightMap->getWidth()*particleStep)/2.0, 0.1, (heightMap->getHeight()*particleStep)/2.0);
        floor->GetCollisionModel()->BuildModel();

        if(visContainer){
            ChSharedPtr<ChBoxShape> floorVis(new ChBoxShape);
            floorVis->GetBoxGeometry().Size = ChVectord((heightMap->getWidth()*particleStep)/2.0, 0.1, (heightMap->getHeight()*particleStep)/2.0);
            floor->AddAsset(floorVis);
        }

        system->AddBody(floor);

        ChBodyPtr wall1(new ChBody(DEFAULT_BODY));
        wall1->SetCollide(true);
        wall1->SetBodyFixed(true);
        wall1->GetMaterialSurface()->SetFriction(1.0);

        ChFrameMoving<> wall1Frame(ChVectord((heightMap->getWidth()*particleStep)/2.0 + 0.05, maxHeight/2.0, 0), QUNIT);
        wall1->ConcatenatePreTransformation(wall1Frame);

        wall1->GetCollisionModel()->ClearModel();
        wall1->GetCollisionModel()->AddBox(0.1, maxHeight/2.0 + 0.05, (heightMap->getHeight()*particleStep)/2.0);
        wall1->GetCollisionModel()->BuildModel();

        if(visContainer){
            ChSharedPtr<ChBoxShape> wall1Vis(new ChBoxShape);
            wall1Vis->GetBoxGeometry().Size = ChVectord(0.1, maxHeight/2.0 + 0.05, (heightMap->getHeight()*particleStep)/2.0);
            wall1->AddAsset(wall1Vis);
        }

        system->AddBody(wall1);

        ChBodyPtr wall2(new ChBody(DEFAULT_BODY));
        wall2->SetCollide(true);
        wall2->SetBodyFixed(true);
        wall2->GetMaterialSurface()->SetFriction(1.0);

        ChFrameMoving<> wall2Frame(ChVectord(-(heightMap->getWidth()*particleStep)/2.0 - 0.05, maxHeight/2.0, 0), QUNIT);
        wall2->ConcatenatePreTransformation(wall2Frame);

        wall2->GetCollisionModel()->ClearModel();
        wall2->GetCollisionModel()->AddBox(0.1, maxHeight/2.0 + 0.05, (heightMap->getHeight()*particleStep)/2.0);
        wall2->GetCollisionModel()->BuildModel();

        if(visContainer){
            ChSharedPtr<ChBoxShape> wall2Vis(new ChBoxShape);
            wall2Vis->GetBoxGeometry().Size = ChVectord(0.1, maxHeight/2.0 + 0.05, (heightMap->getHeight()*particleStep)/2.0);
            wall2->AddAsset(wall2Vis);
        }

        system->AddBody(wall2);

        ChBodyPtr wall3(new ChBody(DEFAULT_BODY));
        wall3->SetCollide(true);
        wall3->SetBodyFixed(true);
        wall3->GetMaterialSurface()->SetFriction(1.0);

        ChFrameMoving<> wall3Frame(ChVectord(0.0, maxHeight/2.0, -(heightMap->getHeight()*particleStep)/2.0 - 0.05), QUNIT);
        wall3->ConcatenatePreTransformation(wall3Frame);

        wall3->GetCollisionModel()->ClearModel();
        wall3->GetCollisionModel()->AddBox((heightMap->getWidth()*particleStep)/2.0, maxHeight/2.0 + 0.05, 0.1);
        wall3->GetCollisionModel()->BuildModel();

        if(visContainer){
            ChSharedPtr<ChBoxShape> wall3Vis(new ChBoxShape);
            wall3Vis->GetBoxGeometry().Size = ChVectord((heightMap->getWidth()*particleStep)/2.0, maxHeight/2.0 + 0.05, 0.1);
            wall3->AddAsset(wall3Vis);
        }

        system->AddBody(wall3);

        ChBodyPtr wall4(new ChBody(DEFAULT_BODY));
        wall4->SetCollide(true);
        wall4->SetBodyFixed(true);
        wall4->GetMaterialSurface()->SetFriction(1.0);

        ChFrameMoving<> wall4Frame(ChVectord(0.0, maxHeight/2.0, (heightMap->getHeight()*particleStep)/2.0 - 0.05), QUNIT);
        wall4->ConcatenatePreTransformation(wall4Frame);

        wall4->GetCollisionModel()->ClearModel();
        wall4->GetCollisionModel()->AddBox((heightMap->getWidth()*particleStep)/2.0, maxHeight/2.0 + 0.05, 0.1);
        wall4->GetCollisionModel()->BuildModel();

        if(visContainer){
            ChSharedPtr<ChBoxShape> wall4Vis(new ChBoxShape);
            wall4Vis->GetBoxGeometry().Size = ChVectord((heightMap->getWidth()*particleStep)/2.0, maxHeight/2.0 + 0.05, 0.1);
            wall4->AddAsset(wall4Vis);
        }

        system->AddBody(wall4);
    }

    double mass = (4.0 / 3.0) * CH_C_PI * pow(particleSize, 3.0) * particleDensity;
    double inertia = pow(particleSize, 2) * mass;

    ChMaterialPtr particleMat(new ChMaterialSurface);
    particleMat->SetFriction(1.7);

    ChSharedPtr<ChSphereShape> shape(new ChSphereShape);
    shape->GetSphereGeometry().rad = particleSize;

    std::cout << "ParticleSystem: Generating terrain..." << std::endl;

    const int numDiv = 4;
    std::vector<ChSharedPtr<ChParticlesClones>> particleSubSystem;

    #ifdef SIM_USE_PARALLEL
    size_t numParticles = 0;
    for(int i=0; i<heightMap->getHeight(); i++){
        for(int j=0; j<heightMap->getWidth(); j++){
            double height = heightMap->getHeights()[j + (i*heightMap->getWidth())] * maxHeight;
            for(double k=0.0; k<height; k+=particleStep){
                //ChVectord pos(-0.5 * heightMap->getWidth() + ChRandom() * heightMap->getWidth(), ChRandom() * heightMap->getHeight(), -0.5 * maxHeight + ChRandom() * maxHeight);
                ChVectord pos(j*particleStep - (heightMap->getWidth()*0.5*particleStep), k, i*particleStep - (heightMap->getHeight()*0.5*particleStep));
                pos += ChVectord(ChRandom(), ChRandom(), ChRandom());
                ChFrameMoving<> bodyFrame(pos, QUNIT);

                ChBodyPtr particle(new ChBody(DEFAULT_BODY));
                particle->SetCollide(true);
                particle->SetBodyFixed(false);
                particle->ConcatenatePreTransformation(bodyFrame);

                particle->GetCollisionModel()->ClearModel();
                particle->GetCollisionModel()->AddSphere(particleSize);
                particle->GetCollisionModel()->BuildModel();

                //double mass = (4.0 / 3.0) * CH_C_PI * pow(particleSize, 3.0) * particleDensity;
                //double inertia = pow(particleSize, 2) * mass;
                particle->SetMass(mass);
                particle->SetInertiaXX(ChVectord(inertia,inertia,inertia));

                //ChMaterialPtr particleMat(new ChMaterialSurface);
                //particleMat->SetFriction(1.7);
                particle->SetMaterialSurface(particleMat);

                //ChSharedPtr<ChSphereShape> shape(new ChSphereShape);
                //shape->GetSphereGeometry().rad = particleSize;

                particle->AddAsset(shape);

                system->AddBody(particle);

                numParticles++;
            }
        }
    }
    #else
    //ChSharedPtr<ChParticlesClones> chParticles(new ChParticlesClones);
    //chParticles->SetCollide(true);
    for(int i=0; i<numDiv; i++){
        ChSharedPtr<ChParticlesClones> clone(new ChParticlesClones);

        clone->GetCollisionModel()->ClearModel();
        clone->GetCollisionModel()->AddSphere(particleSize);
        clone->GetCollisionModel()->BuildModel();

        clone->SetMass(mass);
        clone->SetInertiaXX(ChVectord(inertia,inertia,inertia));

        clone->SetMaterialSurface(particleMat);

        clone->AddAsset(shape);

        particleSubSystem.push_back(clone);
    }

    //chParticles->GetCollisionModel()->ClearModel();
    //chParticles->GetCollisionModel()->AddSphere(particleSize);
    //chParticles->GetCollisionModel()->BuildModel();

    //double mass = (4.0 / 3.0) * CH_C_PI * pow(particleSize, 3.0) * particleDensity;
    //double inertia = pow(particleSize, 2) * mass;
    //chParticles->SetMass(mass);
    //chParticles->SetInertiaXX(ChVectord(inertia,inertia,inertia));

    //ChMaterialPtr particleMat(new ChMaterialSurface);
    //particleMat->SetFriction(1.7);
    //chParticles->SetMaterialSurface(particleMat);

    //ChSharedPtr<ChSphereShape> shape(new ChSphereShape);
    //shape->GetSphereGeometry().rad = particleSize;

    //chParticles->AddAsset(shape);

    size_t numParticles = 0;

    for(int n=0; n<numDiv; n++){
        int iMin = (heightMap->getHeight()/numDiv)*n;
        int iMax = (heightMap->getHeight()/numDiv)*(n+1);

        for(int i=iMin; i<iMax; i++){
            for(int j=0; j<heightMap->getWidth(); j++){
                double height = heightMap->getHeights()[j + (i*heightMap->getWidth())] * maxHeight;
                for(double k=0.0; k<height; k+=particleStep){
                    //ChVectord pos(-0.5 * heightMap->getWidth() + ChRandom() * heightMap->getWidth(), ChRandom() * heightMap->getHeight(), -0.5 * maxHeight + ChRandom() * maxHeight);
                    ChVectord pos(j*particleStep - (heightMap->getWidth()*0.5*particleStep), k, i*particleStep - ((heightMap->getHeight())*0.5*particleStep));
                    pos += ChVectord(ChRandom()*0.05, 0.15, ChRandom()*0.05);
                    //pos += ChVectord(0.0, 0.15, 0.0);

                    ChFrameMoving<> bodyFrame(pos, QUNIT);
                    //chParticles->AddParticle(bodyFrame.GetCoord());
                    particleSubSystem[n]->AddParticle(bodyFrame.GetCoord());
                    numParticles++;
                }
            }
        }
        system->Add(particleSubSystem[n]);
    }

    std::cout << "ParticleSystem: " << numParticles << " particles" << std::endl;
    //system->Add(chParticles);
    //system->Add(chParticles2);
    #endif //SIM_USE_PARALLEL

}

ParticleSystem::~ParticleSystem(){
    _particles.clear();
}
