/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp, Adrian Knobloch
* @copyright  2010,2011,2017 Manfred Kroehnert, Nikolaus Vahrenkamp, Adrian Knobloch
*             GNU Lesser General Public License
*
*/
#pragma once

#include "../Model/Frame.h"
#include "../Model/Primitive.h"
#include "../VirtualRobot.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <functional>
#include <map>

namespace VirtualRobot
{
    class BoundingBox;

    /**
     * @brief The MultipleMutexLockGuard class locks a vector of mutexes in the constructor and unlocks these in the destructor.
     */
    class MultipleMutexLockGuard
    {
    public:
        MultipleMutexLockGuard(const std::vector<std::shared_ptr<std::recursive_mutex>>& mutexList);
        ~MultipleMutexLockGuard();

    protected:
        std::vector<std::shared_ptr<std::recursive_mutex>> mutexList;
    };

    class VIRTUAL_ROBOT_IMPORT_EXPORT Visualization : public Frame, public std::enable_shared_from_this<Visualization>
    {
        friend class VisualizationSet;
        friend class SelectionGroup;
    public:
        struct Color
        {
            Color()
            {
                transparency = 0.0f;
                r = g = b = 0.5f;
            }
            Color(float r, float g, float b, float transparency = 0.0f): r(r), g(g), b(b), transparency(transparency) {}
            float r, g, b;
            float transparency;
            bool isNone() const
            {
                return transparency >= 1.0f;
            }
            bool isTransparencyOnly() const
            {
                return r > 1.f || g > 1.f || b > 1.f || r < 0.f || g < 0.f || b < 0.f;
            }
            inline bool operator==(const Color& c) const
            {
                return (isNone() && c.isNone()) ||
                       (isTransparencyOnly() && c.isTransparencyOnly() && transparency == c.transparency) ||
                       (r == c.r && g == c.g && b == c.b);
            }
            inline bool operator!=(const Color& c) const
            {
                return !(*this == c);
            }

            static Color Blue(float transparency = 0.0f)
            {
                return Color(0.2f, 0.2f, 1.0f, transparency);
            }
            static Color Red(float transparency = 0.0f)
            {
                return Color(1.0f, 0.2f, 0.2f, transparency);
            }
            static Color Green(float transparency = 0.0f)
            {
                return Color(0.2f, 1.0f, 0.2f, transparency);
            }
            static Color Black(float transparency = 0.0f)
            {
                return Color(0, 0, 0, transparency);
            }
            static Color Gray()
            {
                return Color(0.5f, 0.5f, 0.5f, 0);
            }
            static Color None()
            {
                return Color(0.0f, 0.0f, 0.0f, 1.0f);
            }
            static Color Transparency(float transparency)
            {
                return Color(-1.f, -1.f, -1.f, transparency);
            }
            static Color Random()
            {
                float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                float g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                float b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                return Color(r, g, b, 0.0f);
            }
        };

        struct Material
        {
            virtual ~Material() = default;
        };
        using MaterialPtr = std::shared_ptr<Material>;
        struct NoneMaterial : public Material
        {
            virtual ~NoneMaterial() = default;
        };
        struct PhongMaterial : public Material
        {
            virtual ~PhongMaterial() = default;
            Color ambient;
            Color diffuse;
            Color specular;
            float shininess;
            float transparency;
        };
        using PhongMaterialPtr = std::shared_ptr<PhongMaterial>;

        enum DrawStyle
        {
            undefined,
            normal,
            wireframe
        };

    protected:
        /*!
        Constructor
        */
        Visualization();

    public:
        virtual void init();

        /*!
        */
        virtual ~Visualization() override = default;

        /*!
            Sets the position of the internal data structure.
        */
        virtual void setGlobalPose(const Eigen::Matrix4f& m) override;
        virtual void applyDisplacement(const Eigen::Matrix4f& dp);
        virtual size_t addPoseChangedCallback(std::function<void (const Eigen::Matrix4f &)> f);
        virtual void removePoseChangedCallback(size_t id);

        /*!
            Set the visibility of this visualisation.
            \param visible If false, the visualization is not shown.
        */
        virtual void setVisible(bool showVisualization) = 0;
        virtual bool isVisible() const = 0;

        /*!
            Enables/Disables the visualization updates.
            Usually if a model node changes its state, the visualization is automatically updated.
            This behavior can be changed here.
        */
        virtual void setUpdateVisualization(bool enable) = 0;
        virtual bool getUpdateVisualizationStatus() const = 0;

        virtual void setStyle(DrawStyle s) = 0;
        virtual DrawStyle getStyle() const = 0;

        virtual void setColor(const Color& c) = 0;
        virtual Color getColor() const = 0;
        /*!
            Colorize this visualization, but just set the transparency flag (no additional colorization is performed).
            @param transparency The transparent value in [0..1].
        */
        inline void setTransparency(float transparency)
        {
            this->setColor(Color::Transparency(transparency));
        }

        virtual void setMaterial(const MaterialPtr& material) = 0;
        virtual MaterialPtr getMaterial() const = 0;

        inline void select()
        {
            this->setSelected(true);
        }
        inline void deselect()
        {
            this->setSelected(false);
        }
        virtual void setSelected(bool selected);
        virtual bool isSelected() const;
        virtual size_t addSelectionChangedCallback(std::function<void (bool)> f);
        virtual void removeSelectionChangedCallback(size_t id);
    protected:
        virtual void executeSelectionChangedCallbacks(bool selected);
    public:
        virtual void setSelectionGroup(const SelectionGroupPtr& group);
        virtual SelectionGroupPtr getSelectionGroup() const;

        inline void scale(float scaleFactor)
        {
            scale(Eigen::Vector3f(scaleFactor, scaleFactor, scaleFactor));
        }
        virtual void scale(const Eigen::Vector3f& scaleFactor) = 0;

        virtual void shrinkFatten(float offset) = 0;

        virtual std::vector<Primitive::PrimitivePtr> getPrimitives() const = 0;

        //! Just stores the filename, no loading is performed!
        virtual void setFilename(const std::string& filename, bool boundingBox) = 0;
        //! optional filename tag
        virtual std::string getFilename() const = 0;
        virtual bool usedBoundingBoxVisu() const = 0;
        inline std::vector<std::string> getTextureFiles() const
        {
            std::vector<std::string> storeFilenames;
            getTextureFiles(storeFilenames);
            return storeFilenames;
        }
        virtual void getTextureFiles(std::vector<std::string>& storeFilenames) const = 0;

        /*!
            Returns (current) bounding box in global coordinate system.
        */
        virtual BoundingBox getBoundingBox() const = 0;

        /*!
            Returns the TriMeshModel in local coordinate system.
        */
        virtual TriMeshModelPtr getTriMeshModel() const = 0;

        //! get number of faces (i.e. triangles) of this object
        virtual int getNumFaces() const = 0;

        /*!
            Clone this visualization.
            \param deepCopy When true, the underlying visualization is copied, otherwise a reference to the existing visualization is passed.
            \param scaling Scale Can be set to create a scaled version of this visual data.
            Since the underlying implementation may be able to re-use the visualization data, a deep copy may not be necessary in some cases.
        */
        virtual VisualizationPtr clone() const = 0;

        //! print information about this visualization object.
        virtual void print() const = 0;

        virtual std::string toXML(const std::string& basePath, int tabs) const = 0;

        /*!
            Ctreate XML string and replace filename
        */
        virtual std::string toXML(const std::string& basePath, const std::string& filename, int tabs) const = 0;

        /*!
            Saves model file to model path.
            \param modelPath The directory.
        */
        virtual bool saveModel(const std::string& modelPath, const std::string& filename) = 0;

        virtual void addMutex(const std::shared_ptr<std::recursive_mutex>& m);
        virtual void removeMutex(const std::shared_ptr<std::recursive_mutex>& m);

        std::shared_ptr<MultipleMutexLockGuard> getScopedLock() const;

    protected:
        std::map<size_t, std::function<void(const Eigen::Matrix4f&)>> poseChangedCallbacks;
        std::map<size_t, std::function<void(bool)>> selectionChangedCallbacks;
        SelectionGroupPtr selectionGroup;

        mutable std::mutex mutexListChangeMutex;
        mutable std::vector<std::shared_ptr<std::recursive_mutex>> mutexList;
    };

    class VIRTUAL_ROBOT_IMPORT_EXPORT DummyVisualization : virtual public Visualization
    {
        friend class VisualizationFactory;
    protected:
        /*!
        Constructor
        */
        DummyVisualization();

    public:
        virtual void init() override;

        /*!
        */
        virtual ~DummyVisualization() override = default;

        /*!
            Set the visibility of this visualisation.
            \param visible If false, the visualization is not shown.
        */
        virtual void setVisible(bool showVisualization) override;
        virtual bool isVisible() const override;

        /*!
            Enables/Disables the visualization updates.
            Usually if a model node changes its state, the visualization is automatically updated.
            This behavior can be changed here.
        */
        virtual void setUpdateVisualization(bool enable) override;
        virtual bool getUpdateVisualizationStatus() const override;

        virtual void setStyle(DrawStyle s) override;
        virtual DrawStyle getStyle() const override;

        virtual void setColor(const Color& c) override;
        virtual Color getColor() const override;

        virtual void setMaterial(const MaterialPtr& material) override;
        virtual MaterialPtr getMaterial() const override;

        virtual void scale(const Eigen::Vector3f& s) override;

        virtual void shrinkFatten(float offset) override;

        virtual std::vector<Primitive::PrimitivePtr> getPrimitives() const override;

        //! Just stores the filename, no loading is performed!
        virtual void setFilename(const std::string& filename, bool boundingBox) override;
        //! optional filename tag
        virtual std::string getFilename() const override;
        virtual bool usedBoundingBoxVisu() const override;
        virtual void getTextureFiles(std::vector<std::string>& storeFilenames) const override;

        /*!
            Returns (current) bounding box in global coordinate system.
        */
        virtual BoundingBox getBoundingBox() const override;

        virtual TriMeshModelPtr getTriMeshModel() const override;

        //! get number of faces (i.e. triangles) of this object
        virtual int getNumFaces() const override;

        /*!
            Clone this visualization.
            \param deepCopy When true, the underlying visualization is copied, otherwise a reference to the existing visualization is passed.
            \param scaling Scale Can be set to create a scaled version of this visual data.
            Since the underlying implementation may be able to re-use the visualization data, a deep copy may not be necessary in some cases.
        */
        virtual VisualizationPtr clone() const override;

        //! print information about this visualization object.
        virtual void print() const override;

        virtual std::string toXML(const std::string& basePath, int tabs) const override;

        /*!
            Ctreate XML string and replace filename
        */
        virtual std::string toXML(const std::string& basePath, const std::string& filename, int tabs) const override;

        /*!
            Saves model file to model path.
            \param modelPath The directory.
        */
        virtual bool saveModel(const std::string& modelPath, const std::string& filename) override;

    protected:
        //! update trimesh model
        void createTriMeshModel();

        bool visible;
        bool updateVisualization;
        DrawStyle style;
        Color color;
        MaterialPtr material;
        std::string filename; //!< if the visualization was build from a file, the filename is stored here
        bool boundingBox; //!< Indicates, if the bounding box model was used
        std::vector<Primitive::PrimitivePtr> primitives;
        TriMeshModelPtr triMeshModel;
    };

} // namespace VirtualRobot

