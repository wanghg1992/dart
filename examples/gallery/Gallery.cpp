/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Gallery.hpp"

#include "EventHandler.hpp"
#include "MainMenuWidget.hpp"
#include "OsgProjectNode.hpp"
#include "OutputWidget.hpp"
#include "ProjectExplorerWidget.hpp"
#include "ProjectWidget.hpp"
#include "rigidCubes/RigidCubesProject.hpp"
#include "boxStacking/BoxStackingProject.hpp"

namespace dart {
namespace examples {

//==============================================================================
Gallery::Gallery() : mProjectTreeRoot(""), mCurrentProject(nullptr)
{
  viewer.setKeyEventSetsDone(0);

  mMainMenuWidget = std::make_shared<MainMenuWidget>(this, &viewer);
  mProjectExplorerWidget
      = std::make_shared<ProjectExplorerWidget>(this, &viewer);
  mProjectWidget = std::make_shared<ProjectWidget>(this, &viewer);
  mOutputWidget = std::make_shared<OutputWidget>(this, &viewer);

  viewer.getImGuiHandler()->addWidget(mMainMenuWidget);
  viewer.getImGuiHandler()->addWidget(mProjectExplorerWidget);
  viewer.getImGuiHandler()->addWidget(mProjectWidget);
  viewer.getImGuiHandler()->addWidget(mOutputWidget);

  viewer.addEventHandler(new EventHandler);

  viewer.setUpViewInWindow(0, 0, 1280, 720);

  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.57f, 3.14f, 1.64f),
      ::osg::Vec3(0.00f, 0.00f, 0.00f),
      ::osg::Vec3(-0.24f, -0.25f, 0.94f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  auto oldProjects = ProjectGroup::create("Old Projects");
  mProjectTreeRoot.addChild(oldProjects);
  oldProjects->addChild(TProjectNote<RigidCubesProject>::create());
  oldProjects->addChild(TProjectNote<RigidCubesProject>::create());
  oldProjects->addChild(TProjectNote<BoxStackingProject>::create());
}

//==============================================================================
void Gallery::run()
{
  viewer.run();
}

//==============================================================================
void Gallery::selectProject(const ProjectNode* node)
{
  bool simulating = viewer.isSimulating();
  if (mCurrentProject)
  {
    std::cout << "[DEBUG] 1" << std::endl;
    if (simulating)
      viewer.simulate(false);
    std::cout << "[DEBUG] 2" << std::endl;
    mCurrentProject->finalize();
    std::cout << "[DEBUG] 3" << std::endl;
//    viewer.removeWorldNode(mCurrentProject->getOsgNode());
    viewer.setWorldNodeActive(mCurrentProject->getOsgNode(), false);
    std::cout << "[DEBUG] 4" << std::endl;
    mCurrentProject = nullptr;
  }
  viewer.realize();
  //viewer.setDone(true);

  std::cout << "[DEBUG] 5" << std::endl;
  if (!node)
    return;

  std::cout << "[DEBUG] 6" << std::endl;
  auto createFunction = node->getCreateFunction();
  if (!createFunction)
    return;

  std::cout << "[DEBUG] 7" << std::endl;
  mCurrentProject = createFunction();

  if (!mCurrentProject)
  {
    dtwarn << "[Gallery] Failed to create project.\n";
    return;
  }

  std::cout << "[DEBUG] 8" << std::endl;
  mCurrentProject->initialize();

  std::cout << "[DEBUG] 9" << std::endl;
  auto osgNode = mCurrentProject->getOsgNode();
  if (!osgNode)
  {
    dtwarn << "[Gallery] Failed to get OSG node from project. This will lead "
           << "to rendering nothing and simulating nothing.\n";
    return;
  }

  std::cout << "[DEBUG] 10" << std::endl;
  viewer.addWorldNode(osgNode);

  std::cout << "[DEBUG] 11" << std::endl;
  std::stringstream ss;
  ss << "Project '" << mCurrentProject->getName() << "' is loaded.\n";
  mOutputWidget->addLog(ss.str());

  std::cout << "[DEBUG] 11" << std::endl;
}

//==============================================================================
Project* Gallery::getCurrentProject()
{
  return mCurrentProject.get();
}

//==============================================================================
const Project* Gallery::getCurrentProject() const
{
  return mCurrentProject.get();
}

//==============================================================================
ProjectGroup* Gallery::getProjectTreeRoot()
{
  return &mProjectTreeRoot;
}

//==============================================================================
const ProjectGroup* Gallery::getProjectTreeRoot() const
{
  return &mProjectTreeRoot;
}

//==============================================================================
MainMenuWidget* Gallery::getMainMenuWidget()
{
  return mMainMenuWidget.get();
}

//==============================================================================
ProjectExplorerWidget* Gallery::getProjectExplorerWidget()
{
  return mProjectExplorerWidget.get();
}

//==============================================================================
ProjectWidget* Gallery::getProjectWidget()
{
  return mProjectWidget.get();
}

//==============================================================================
OutputWidget* Gallery::getOutputWidget()
{
  return mOutputWidget.get();
}

} // namespace examples
} // namespace dart
