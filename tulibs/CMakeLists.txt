cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  Tulibs
  VERSION 0.1
  RELEASE alpha
  SUMMARY "A collection of C-based Tool and Utility LIBrarieS."
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://aslforge.ethz.ch/projects/robotks"
  LICENSE "GNU Lesser General Public License (LGPL)"
  INSTALL /usr
)

remake_doc(
  html
)

remake_pack_deb()
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  tulibs
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)
