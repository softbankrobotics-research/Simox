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
 * SoGLHighlightRenderAction.cpp
 *
 *  Created on: Jan 13, 2015
 *      Author: philipp
 */

#include "SoGLHighlightRenderAction.h"
#include "../AbstractViewer.h"

SimoxGui::SoGLHighlightRenderAction::SoGLHighlightRenderAction(const SbViewportRegion& viewportregion) :
    SoGLRenderAction(viewportregion), selectedcolor_storage(sizeof(void*), alloc_colorpacker, free_colorpacker)
{

    this->isVisible = true;
    this->selectedColor = SbColor(1.0f, 0.8f, 0.4f);
    this->activeColor = SbColor(1.0f, 0.4f, 0.2f);
    this->linepattern = 0xffff;
    this->linewidth = 5;

    // SoBase-derived objects should be dynamically allocated.
    this->postprocpath = new SoTempPath(32);
    this->postprocpath->ref();

}

SimoxGui::SoGLHighlightRenderAction::~SoGLHighlightRenderAction()
{
    postprocpath->unref();
}

void SimoxGui::SoGLHighlightRenderAction::apply(SoNode* node)
{
    //Render complete scene with standard action
    SoGLRenderAction::apply(node);

    if (this->isVisible)
    {
        SoSearchAction* searchAction = new SoSearchAction;
        searchAction->setType(SoSelection::getClassTypeId());
        searchAction->setInterest(SoSearchAction::ALL);
        searchAction->apply(node);

        const SoPathList& pathlist = searchAction->getPaths();

        if (pathlist.getLength() > 0)
        {
            for (int i = 0; i < pathlist.getLength(); i++)
            {
                SoPath* path = pathlist[i];
                SoSelection* selection = (SoSelection*) path->getTail();

                if (selection->getNumSelected() > 0)
                    //Add highlight border around selected objects
                {
                    this->drawHighlight(path, selection->getList());
                }
            }
        }
    }
}

void SimoxGui::SoGLHighlightRenderAction::drawHighlight(SoPath* pathtothis,
                                                        const SoPathList* pathlist)
{
    // Copied from Coin implementation, no idea what it does
    int oldnumpasses = this->getNumPasses();
    this->setNumPasses(1);

    //Insert path to SoSelection node in our postprocesspath
    int thispos = ((SoFullPath*) pathtothis)->getLength() - 1;
    this->postprocpath->truncate(0);

    for (int i = 0; i < thispos; i++)
    {
        this->postprocpath->append(pathtothis->getNode(i));
    }

    //For 0 to n - 1 selected object of selection list render colored wireframe
    for (int i = 0; i < pathlist->getLength(); i++)
    {
        //Retrieve full path
        SoFullPath* path = (SoFullPath*)(*pathlist)[i];

        //Append to postprocesspath
        for (int j = 0; j < path->getLength(); j++)
        {
            this->postprocpath->append(path->getNode(j));
        }

        /*
          We use a little algorithm mostly explained here:
          http://www.flipcode.com/archives/Object_Outlining.shtml
          and here:
          http://www.informit.com/articles/article.aspx?p=328646&seqNum=10

          We basically fill the opengl stencil buffer with the value 1,
          tell opengl to never touch the color buffer but modify the stencil buffer only.
          (Actually writing zero into it at the respective pixel positions).
          In the next step we render the object again, this time drawing
          the wireframe version of the object with the selection color
          and only drawing where we still have the value 1 in stencil buffer.
          Because line width is 5, we actually get some sort of 'edge glow'
          effect out of this. :)
        */

        // Use 1 for clearing stencil buffer, clear buffer and enable stencil test
        glEnable(GL_STENCIL_TEST);
        glClearStencil(1.0f);
        glClear(GL_STENCIL_BUFFER_BIT);

        // All drawing commands fail the stencil test and are not
        // drawn, but set the value in the stencil buffer to zero.
        glStencilFunc(GL_NEVER, 0x0, 0x0);
        glStencilOp(GL_ZERO, GL_ZERO, GL_ZERO);
        SoGLRenderAction::apply(this->postprocpath);

        //Render only where stencil is not set to 0, to achieve sort of 'edge glow' effect
        glStencilFunc(GL_NOTEQUAL, 0, 1);
        //Do not modify stencil buffer in any way (just to keep it clean, doesn't really matter)
        glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

        //Draw wireframe in selection color, masked by stencil
        drawWireframe(this->postprocpath, i == pathlist->getLength() - 1);

        //Reset postprocesspath to SoSelection node
        this->postprocpath->truncate(thispos);

        glDisable(GL_STENCIL_TEST);
    }

    //Restore this mysterious numpass thing
    this->setNumPasses(oldnumpasses);
}

void SimoxGui::SoGLHighlightRenderAction::drawWireframe(SoPath* pathtothis, bool active)
{
    //Save current openGL state
    SoState* state = this->getState();
    state->push();

    //Cause Coin sucks we have to use OpenGL calls directly here
    //because calling SoLazyElement::setLightModel(state, SoLazyElement::BASE_COLOR)
    //ends in SEGFAULT for some selected objects
    glDisable(GL_LIGHTING);

    //Do some pretty complex and fun OpenGL adaptions
    SoColorPacker** selected_cptr =
        (SoColorPacker**) this->selectedcolor_storage.get();

    if (!active)
    {
        SoGLLazyElement::setDiffuse(state, pathtothis->getHead(), 1,
                                    &this->selectedColor, *selected_cptr);
    }
    else
    {
        //Set new color for last object
        SoGLLazyElement::setDiffuse(state, pathtothis->getTail(), 1,
                                    &this->activeColor, *selected_cptr);
    }

    SoLineWidthElement::set(state, this->linewidth);
    SoLinePatternElement::set(state, this->linepattern);
    SoTextureQualityElement::set(state, 0.0f);
    SoDrawStyleElement::set(state, SoDrawStyleElement::LINES);
    SoPolygonOffsetElement::set(state, NULL, -1.0f, 1.0f,
                                SoPolygonOffsetElement::LINES, TRUE);
    SoMaterialBindingElement::set(state, NULL,
                                  SoMaterialBindingElement::OVERALL);
    SoNormalElement::set(state, NULL, 0, NULL, FALSE);
    SoOverrideElement::setNormalVectorOverride(state, NULL, TRUE);
    SoOverrideElement::setMaterialBindingOverride(state, NULL, TRUE);
    SoOverrideElement::setLightModelOverride(state, NULL, TRUE);
    SoOverrideElement::setDiffuseColorOverride(state, NULL, TRUE);
    SoOverrideElement::setLineWidthOverride(state, NULL, TRUE);
    SoOverrideElement::setLinePatternOverride(state, NULL, TRUE);
    SoOverrideElement::setDrawStyleOverride(state, NULL, TRUE);
    SoOverrideElement::setPolygonOffsetOverride(state, NULL, TRUE);
    SoTextureOverrideElement::setQualityOverride(state, TRUE);

    //Render with new OpenGL context
    SoGLRenderAction::apply(this->postprocpath);

    //Cause switching light on again might be a good idea
    glEnable(GL_LIGHTING);

    //Restore OpenGL context
    state->pop();
}


