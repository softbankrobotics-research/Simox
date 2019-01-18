/*
 * This file is part of ArmarX.
 *
 * Copyright (C) 2012-2016, High Performance Humanoid Technologies (H2T), Karlsruhe Institute of Technology (KIT), all rights reserved.
 *
 * ArmarX is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ArmarX is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @package    MemoryX::gui-plugins::SceneEditor
 * @date       2015
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

/*
 * SoGLHighlightRenderAction.h
 *
 *  Created on: Jan 13, 2015
 *      Author: philipp
 */

#pragma once

#include <Inventor/actions/SoGLRenderAction.h>
#include <Inventor/SbName.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/actions/SoSubAction.h>
#include <Inventor/elements/SoDrawStyleElement.h>
#include <Inventor/elements/SoGLLazyElement.h>
#include <Inventor/elements/SoLinePatternElement.h>
#include <Inventor/elements/SoLineWidthElement.h>
#include <Inventor/elements/SoOverrideElement.h>
#include <Inventor/elements/SoPolygonOffsetElement.h>
#include <Inventor/elements/SoTextureOverrideElement.h>
#include <Inventor/elements/SoTextureQualityElement.h>
#include <Inventor/elements/SoMaterialBindingElement.h>
#include <Inventor/elements/SoNormalElement.h>
#include <Inventor/lists/SoEnabledElementsList.h>
#include <Inventor/lists/SoPathList.h>
#include <Inventor/misc/SoState.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/threads/SbStorage.h>

#include <mutex>
#include <memory>

namespace SimoxGui
{
    class AbstractViewer;
    class SoGLHighlightRenderAction : public SoGLRenderAction
    {
    public:
        /**
         * Constructor
         * Creates an Instance of the Class
         *
         * @param viewportregion ViewportRegion the Class is initialized with
         */
        SoGLHighlightRenderAction(const SbViewportRegion& viewportregion, AbstractViewer* viewer);

        /**
         * Destructor.
         *
         */
        ~SoGLHighlightRenderAction() override;

        /**
         * Applies the Highlighting to a given node.
         *
         * @param node Node to be highlighted
         */
        void apply(SoNode* node) override;

        /**
         * Sets the Highlighting visible.
         *
         */
        void setVisible();

        /**
         * Returns if the Highlighting is visible.
         *
         * @return bool Visibility of Highlighting
         */
        bool getVisible();

    private:
        AbstractViewer* viewer;

        bool isVisible;

        void drawHighlight(SoPath* pathtothis, const SoPathList* pathlist);
        void drawWireframe(SoPath* pathtothis, bool active);

        SbColor selectedColor;
        SbColor activeColor;
        uint16_t linepattern;
        float linewidth;
        SoTempPath* postprocpath;
        SbStorage selectedcolor_storage;

        static void alloc_colorpacker(void* data)
        {
            SoColorPacker** cptr = (SoColorPacker**) data;
            *cptr = new SoColorPacker;
        }

        static void free_colorpacker(void* data)
        {
            SoColorPacker** cptr = (SoColorPacker**) data;
            delete *cptr;
        }
    };
}

