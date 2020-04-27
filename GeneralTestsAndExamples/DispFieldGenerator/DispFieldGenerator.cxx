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
//    INPUTS: CT.nrrd T1.nrrd LinearTransform.h5

#include "itkVersorRigid3DTransform.h"
#include "itkAffineTransform.h"

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
  using TransformReaderType = itk::TransformFileReaderTemplate<float>;
  TransformReaderType::Pointer transformReader = TransformReaderType::New();
  transformReader->SetFileName(argv[3]);
  transformReader->Update();

  using TransformType = itk::AffineTransform<float, Dimension>;
  TransformType::Pointer transform = TransformType::New();

  TransformReaderType::TransformListType * transforms = transformReader->GetTransformList();
  auto it = transforms->begin();

  std::cout << "we got heare!" << std::endl;
  if (!strcmp((*it)->GetNameOfClass(), "AffineTransform")){
      std::cout << "we got hear too!" << std::endl;
      transform = static_cast<TransformType *>((*it).GetPointer());
      transform->Print(std::cout);
  }

  //std::cout<< "Name of tfm:" << (*it)->GetNameOfClass() <<"Transform category"<<(*it).GetPointer()->GetTransformCategory()<< std::endl;
  //std::cout<< "Name of tfm:" << transform->GetNameOfClass() <<"Transform category"<<transform.GetPointer()->GetTransformCategory()<< std::endl;

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
  FieldIteratorType fi( field, fixedImageReader->GetOutput()->GetBufferedRegion());
  fi.GoToBegin();

  TransformType::InputPointType movingPoint;
  TransformType::OutputPointType transformedPoint;
  DisplacementFieldType::IndexType index;

  VectorType displacement;

  while( !fi.IsAtEnd()){
      index = fi.GetIndex();
      field->TransformIndexToPhysicalPoint( index, movingPoint );
      transformedPoint = transform->TransformPoint( movingPoint );
      displacement = transformedPoint - movingPoint;
      fi.Set(displacement);
      ++fi;
  }
  std::cout << "We got here!!!" << std::endl;

  using FieldWriterType = itk::ImageFileWriter<DisplacementFieldType>;
  FieldWriterType::Pointer fieldWriter = FieldWriterType::New();
  fieldWriter->SetInput(field);
  fieldWriter->SetFileName("dispField-q=.nrrd");
  fieldWriter->Update();

  return EXIT_SUCCESS;
}
