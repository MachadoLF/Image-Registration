/*=========================================================================
 *
 *  Copyright Insight Software Consortium
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0.txt
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *=========================================================================*/

//  Software Guide : BeginCommandLineArgs
//    INPUTS:  {FixedImage.png}
//    INPUTS:  {MovingImage.png}
//    INPUTS:  {ImagePath}
//    OUTPUTS: {performance.csv}

//  This algorithm intends to find the optmum q-value
//  for a 3-D rigid multy-modality registration

#include "itkVersorRigid3DTransform.h"

#include <iostream>
#include <fstream>

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

#include "itkTransformFileReader.h"
#include "itkImageRegionIterator.h"

#include "iomanip"


int main( int argc, char *argv[] )
{
  if( argc < 4 )
    {
    std::cerr << " Missing Parameters " << std::endl;
    std::cerr << " Usage: " << argv[0];
    std::cerr << " FixedImageFile  MovingImageFile Transform "<< std::endl;
    return EXIT_FAILURE;
    }

  const    unsigned int    Dimension = 3;
  typedef  float           PixelType;

  typedef itk::Image< PixelType, Dimension >  FixedImageType;
  typedef itk::Image< PixelType, Dimension >  MovingImageType;

  typedef itk::ImageFileReader< FixedImageType  > FixedImageReaderType;
  typedef itk::ImageFileReader< MovingImageType > MovingImageReaderType;

  FixedImageReaderType::Pointer  fixedImageReader  = FixedImageReaderType::New();
  MovingImageReaderType::Pointer movingImageReader = MovingImageReaderType::New();

  fixedImageReader->SetFileName(  argv[1] );
  movingImageReader->SetFileName( argv[2] );

  // Writing Transform
  using TransformReaderType = itk::TransformFileReader;
  TransformReaderType::Pointer transformReader = TransformReaderType::New();
  transformReader->SetFileName(argv[3]);
  transformReader->Update();

  TransformReaderType::TransformListType * transforms = transformReader->GetTransformList();

  // Creating and writing the displacement vector image
  // Based on example: https://itk.org/Doxygen/html/Examples_2RegistrationITKv3_2DeformableRegistration8_8cxx-example.html

  using VectorType = itk::Vector<float, Dimension>;
  using DisplacementFieldType = itk::Image<VectorType, Dimension>;
  DisplacementFieldType::Pointer field = DisplacementFieldType::New();
  field->SetRegions(fixedImageReader->GetOutput()->GetBufferedRegion());
  field->SetSpacing(fixedImageReader->GetOutput()->GetSpacing());
  field->SetOrigin(fixedImageReader->GetOutput()->GetOrigin());
  field->SetDirection(fixedImageReader->GetOutput()->GetDirection());
  field->Allocate();

  using FieldIteratorType = itk::ImageRegionIterator<DisplacementFieldType>;
  FieldIteratorType fi( field, movingImageReader->GetOutput()->GetBufferedRegion());
  fi.GoToBegin();

  using TransformType = itk::VersorRigid3DTransform<double>;
  TransformType::InputPointType movingPoint;
  TransformType::OutputPointType transformedPoint;
  DisplacementFieldType::IndexType index;

  VectorType displacement;

  while( !fi.IsAtEnd()){
      index = fi.GetIndex();
      field->TransformIndexToPhysicalPoint( index, movingPoint );
      transforms->begin();
      transformedPoint = transforms-> ->TransformPoint( movingPoint );
      displacement = transformedPoint - movingPoint;
      fi.Set(displacement);
      ++fi;
  }

  using FieldWriterType = itk::ImageFileWriter<DisplacementFieldType>;
  FieldWriterType::Pointer fieldWriter = FieldWriterType::New();
  fieldWriter->SetInput(field);
  fieldWriter->SetFileName(imagePath + "dispField-q=" + id + ".nrrd");
  fieldWriter->Update();

  return EXIT_SUCCESS;
}
