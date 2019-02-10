#include "OffscreenRenderEngineShaderEffect.h"

#include <Qt3DRender/QGraphicsApiFilter>
#include <Qt3DRender/QShaderProgram>
#include <QUrl>

VirtualRobot::OffscreenRenderEngineShaderEffect::OffscreenRenderEngineShaderEffect(const QString& name, const QUrl& vertexShader, const QUrl& fragmentShader, Qt3DCore::QNode *parent)
    : Qt3DRender::QEffect(parent)
    , technique(new Qt3DRender::QTechnique(this))
    , pass(new Qt3DRender::QRenderPass(this))
    , passCriterion(new Qt3DRender::QFilterKey(this))
{

    technique->graphicsApiFilter()->setProfile(Qt3DRender::QGraphicsApiFilter::CoreProfile);
    technique->graphicsApiFilter()->setApi(Qt3DRender::QGraphicsApiFilter::OpenGL);
    technique->graphicsApiFilter()->setMajorVersion(3);
    technique->graphicsApiFilter()->setMinorVersion(0);

    passCriterion->setName(name);
    passCriterion->setValue(QStringLiteral("pass"));

    Qt3DRender::QShaderProgram *shader = new Qt3DRender::QShaderProgram();
    shader->setVertexShaderCode(Qt3DRender::QShaderProgram::loadSource(vertexShader));
    shader->setFragmentShaderCode(Qt3DRender::QShaderProgram::loadSource(fragmentShader));

    pass->addFilterKey(passCriterion);
    pass->setShaderProgram(shader);
    technique->addRenderPass(pass);

    addTechnique(technique);
}

QList<Qt3DRender::QFilterKey *> VirtualRobot::OffscreenRenderEngineShaderEffect::passCriteria() const
{
    return QList<Qt3DRender::QFilterKey *>() << passCriterion;
}
