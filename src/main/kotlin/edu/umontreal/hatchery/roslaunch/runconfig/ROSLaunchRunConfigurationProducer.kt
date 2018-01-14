package edu.umontreal.hatchery.roslaunch.runconfig;

import com.intellij.execution.actions.ConfigurationContext
import com.intellij.execution.actions.RunConfigurationProducer
import com.intellij.openapi.util.Ref
import com.intellij.psi.PsiElement

class ROSLaunchRunConfigurationProducer: RunConfigurationProducer<ROSLaunchRunConfiguration>(ROSLaunchRunConfigurationType()){
    override fun isConfigurationFromContext(configuration: ROSLaunchRunConfiguration?, context: ConfigurationContext?): Boolean {
        TODO("not yet implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setupConfigurationFromContext(configuration: ROSLaunchRunConfiguration?, context: ConfigurationContext?, sourceElement: Ref<PsiElement>?): Boolean {
        TODO("not yet implemented") //To change body of created functions use File | Settings | File Templates.
    }
}

