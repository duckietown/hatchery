package edu.umontreal.hatchery

import org.gradle.api.model.ObjectFactory

import org.gradle.kotlin.dsl.*


open class MyProjectExtension(objects: ObjectFactory) {

    val flag = objects.property<Boolean>()
}