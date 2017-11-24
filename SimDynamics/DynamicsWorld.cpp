
#include "DynamicsWorld.h"
#include "DynamicsEngine/DynamicsEngineFactory.h"


namespace SimDynamics
{


    namespace
    {
        std::mutex mutex;
    }

    DynamicsWorldPtr DynamicsWorld::world;

#if 0
    bool DynamicsWorld::convertMM2M = false;
#else
    bool DynamicsWorld::convertMM2M = true;
#endif

    DynamicsWorld::Cleanup::~Cleanup()
    {
        std::lock_guard<std::mutex> lock(mutex);
        DynamicsWorld::world.reset();
    }


    DynamicsWorldPtr DynamicsWorld::GetWorld()
    {
        if (!world)
        {
            Init();
        }

        return world;
    }


    DynamicsEnginePtr DynamicsWorld::getEngine()
    {
        return engine;
    }

    DynamicsWorldPtr DynamicsWorld::Init(DynamicsEngineConfigPtr config)
    {
        static Cleanup _Cleanup;

        if (true)
        {
            std::lock_guard<std::mutex> lock(mutex);

            if (!world)
            {
                world.reset(new DynamicsWorld(config));
            }
            else
            {
                VR_WARNING << "Dynamics world is already initialized..." << endl;
            }
        }

        return world;
    }



    void DynamicsWorld::Close()
    {
        world.reset();
    }


    DynamicsWorld::DynamicsWorld(DynamicsEngineConfigPtr config)
    {
        DynamicsEngineFactoryPtr factory = DynamicsEngineFactory::first(nullptr);
        THROW_VR_EXCEPTION_IF(!factory, "No Physics Engine Found. Re-Compile with engine support...");
        engine = factory->createEngine(config);
        THROW_VR_EXCEPTION_IF(!engine, "Could not create Physics Engine.");
    }

    DynamicsWorld::~DynamicsWorld()
    {
    }

    bool DynamicsWorld::addObject(DynamicsObjectPtr o)
    {
        return engine->addObject(o);
    }

    bool DynamicsWorld::removeObject(DynamicsObjectPtr o)
    {
        return engine->removeObject(o);
    }

    DynamicsObjectPtr DynamicsWorld::CreateDynamicsObject(VirtualRobot::ModelLinkPtr o)
    {
        SIMDYNAMICS_ASSERT(o);

        DynamicsEngineFactoryPtr factory = DynamicsEngineFactory::first(nullptr);
        SIMDYNAMICS_ASSERT(factory);

        return factory->createObject(o);
    }

    void DynamicsWorld::createFloorPlane(const Eigen::Vector3f& pos /*= Eigen::Vector3f(0,0,0)*/, const Eigen::Vector3f& up /*= Eigen::Vector3f(0,0,1.0f)*/)
    {
        engine->createFloorPlane(pos, up);
    }

    bool DynamicsWorld::addModel(DynamicsModelPtr r)
    {
        return engine->addModel(r);
    }

    bool DynamicsWorld::removeRobot(DynamicsModelPtr r)
    {
        return engine->removeRobot(r);
    }

    SimDynamics::DynamicsModelPtr DynamicsWorld::CreateDynamicsModel(const VirtualRobot::ModelPtr &rob)
    {
        SIMDYNAMICS_ASSERT(rob);

        DynamicsEngineFactoryPtr factory = DynamicsEngineFactory::first(nullptr);
        SIMDYNAMICS_ASSERT(factory);

        return factory->createRobot(rob);
    }

    std::vector<DynamicsModelPtr> DynamicsWorld::getRobots()
    {
        return engine->getRobots();
    }

} // namespace
