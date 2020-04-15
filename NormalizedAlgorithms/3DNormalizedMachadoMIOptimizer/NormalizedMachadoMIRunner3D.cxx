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

// Software Guide : BeginCodeSnippet
#include "itkImageRegistrationMethodv4.h"
#include "itkTranslationTransform.h"
#include "itkNormalizedMachadoMutualInformationImageToImageMetricv4.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"
// Software Guide : EndCodeSnippet

#include "itkVersorRigid3DTransform.h"
#include "itkCenteredTransformInitializer.h"

#include <iostream>
#include <fstream>

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkCheckerBoardImageFilter.h"
#include "itkMersenneTwisterRandomVariateGenerator.h"

#include "itkRegistrationParameterScalesFromIndexShift.h"
#include "itkTransformFileWriter.h"
#include "itkImageRegionIterator.h"

#include "iomanip"


using myFileType = std::ofstream;
static myFileType myfile;

//  The following section of code implements a Command observer
//  used to monitor the evolution of the registration process.
//
#include "itkCommand.h"
class CommandIterationUpdate : public itk::Command
{
public:
  typedef  CommandIterationUpdate   Self;
  typedef  itk::Command             Superclass;
  typedef itk::SmartPointer<Self>   Pointer;
  itkNewMacro( Self )

protected:
  CommandIterationUpdate() {}

public:
  typedef itk::RegularStepGradientDescentOptimizerv4<double> OptimizerType;
  typedef   const OptimizerType *                            OptimizerPointer;

  void Execute(itk::Object *caller, const itk::EventObject & event) ITK_OVERRIDE
    {
    Execute( static_cast<const itk::Object *>(caller), event);
    }

  void Execute(const itk::Object * object, const itk::EventObject & event) ITK_OVERRIDE
    {
    OptimizerPointer optimizer = static_cast< OptimizerPointer >( object );
    if( ! itk::IterationEvent().CheckEvent( &event ) )
      {
      return;
      }
    std::cout << optimizer->GetCurrentIteration() << "   ";
    std::cout << optimizer->GetValue() << "   ";
    std::cout << optimizer->GetCurrentPosition() << std::endl;

    }
};

//for finding maximum

int main( int argc, char *argv[] )
{
  if( argc < 4 )
    {
    std::cerr << " Missing Parameters " << std::endl;
    std::cerr << " Usage: " << argv[0];
    std::cerr << " FixedImageFile  MovingImageFile ImagePath "<< std::endl;
    return EXIT_FAILURE;
    }

  const    unsigned int    Dimension = 3;
  typedef  float           PixelType;

  typedef itk::Image< PixelType, Dimension >  FixedImageType;
  typedef itk::Image< PixelType, Dimension >  MovingImageType;

  //typedef itk::TranslationTransform< double, Dimension >         TransformType;
  using TransformType = itk::VersorRigid3DTransform< double >;
  typedef itk::RegularStepGradientDescentOptimizerv4<double>     OptimizerType;
  typedef itk::ImageRegistrationMethodv4<
                                    FixedImageType,
                                    MovingImageType,
                                    TransformType    > RegistrationType;

  std::cerr << "Chegou Aqui! " << std::endl;
  std::string imagePath = argv[3];
  myfile.open (imagePath + "performance.csv");


  // Defining specific metric
  // Tsallis
  typedef itk::NormalizedMachadoMutualInformationImageToImageMetricv4< FixedImageType,MovingImageType > MetricType;

  // Mattes
  // typedef itk::MattesMutualInformationImageToImageMetricv4< FixedImageType,MovingImageType > MetricType;
  // Software Guide : EndCodeSnippet


  typedef itk::ImageFileReader< FixedImageType  > FixedImageReaderType;
  typedef itk::ImageFileReader< MovingImageType > MovingImageReaderType;

  FixedImageReaderType::Pointer  fixedImageReader  = FixedImageReaderType::New();
  MovingImageReaderType::Pointer movingImageReader = MovingImageReaderType::New();

  fixedImageReader->SetFileName(  argv[1] );
  movingImageReader->SetFileName( argv[2] );

  // Number of qValues tested
  unsigned int numOfInstances = 1;
  int n_times = 1;

  myfile <<"# All the points presented here are averaged over "<<n_times<<" times execution."<<std::endl;
  myfile <<"qValue,Number of Iterations,X,Y,Z,Angle"<<std::endl;

  double qValueIni = 0.5;
  for (unsigned int i = 0; i <= numOfInstances; ++i) // qValue Loop
  {
      double meanNumberOfIterations = 0.0;
      double meanX = 0.0;
      double meanY = 0.0;
      double meanZ = 0.0;
      double meanAngle = 0.0;

      double qValue = qValueIni + i;

      if (qValue == 1.0){
          continue;
      }

      for (int y = 0; y < n_times; ++y) { // Average x times loop

          RegistrationType::Pointer   registration  = RegistrationType::New();

          OptimizerType::Pointer      optimizer     = OptimizerType::New();

          registration->SetOptimizer(     optimizer     );
          // Setting images

          // Setting Metric
          MetricType::Pointer metric = MetricType::New();
          metric->SetqValue(qValue);
          registration->SetMetric( metric  );

          //  The metric requires the user to specify the number of bins
          //  used to compute the entropy. In a typical application, 50 histogram bins
          //  are sufficient.

          unsigned int numberOfBins = 24;

          metric->SetNumberOfHistogramBins( numberOfBins );

          metric->SetUseMovingImageGradientFilter( false );
          metric->SetUseFixedImageGradientFilter( false );

          // Using the whole image in the Mutual information calculation
          metric->SetUseSampledPointSet( false );

          registration->SetFixedImage(    fixedImageReader->GetOutput()    );
          registration->SetMovingImage(   movingImageReader->GetOutput()   );

          // Initial transform and parameter scales
          TransformType::Pointer  initialTransform = TransformType::New();

          using TransformInitializerType = itk::CenteredTransformInitializer<
              TransformType,
              FixedImageType,
              MovingImageType >;

              TransformInitializerType::Pointer initializer =
              TransformInitializerType::New();

          initializer->SetTransform(   initialTransform );
          initializer->SetFixedImage(  fixedImageReader->GetOutput() );
          initializer->SetMovingImage( movingImageReader->GetOutput() );

          initializer->GeometryOn();

          initializer->InitializeTransform();

          using VersorType = TransformType::VersorType;
          using VectorType = VersorType::VectorType;
          VersorType     rotation;
          VectorType     axis;
          axis[0] = 0.0;
          axis[1] = 0.0;
          axis[2] = 1.0;
          constexpr double angle = 0;
          rotation.Set(  axis, angle  );
          initialTransform->SetRotation( rotation );

          registration->SetInitialTransform( initialTransform );

          // Parameter scale setter

          using ParameterEstimatorType = itk::RegistrationParameterScalesFromPhysicalShift< MetricType >;
          ParameterEstimatorType::Pointer estimator = ParameterEstimatorType::New();
          estimator->SetMetric(metric);
          estimator->SetTransformForward(true);

          /*
          using OptimizerScalesType = OptimizerType::ScalesType;
          OptimizerScalesType optimizerScales( initialTransform->GetNumberOfParameters() );
          const double translationScale = 1.0 / 1000.0;
          optimizerScales[0] = 1.0;
          optimizerScales[1] = 1.0;
          optimizerScales[2] = 1.0;
          optimizerScales[3] = translationScale;
          optimizerScales[4] = translationScale;
          optimizerScales[5] = translationScale;
          */

          optimizer->SetScalesEstimator(estimator);

          // Configuring the optimizer

          optimizer->SetDoEstimateLearningRateOnce(false);
          optimizer->SetDoEstimateLearningRateAtEachIteration(false);

          optimizer->SetLearningRate( 0.2 );
          optimizer->SetMinimumStepLength( 0.001 );
          optimizer->SetNumberOfIterations( 300 );
          optimizer->ReturnBestParametersAndValueOn();

          // Let's avoid setting Tolerance and RelaxationFactor
          //optimizer->SetGradientMagnitudeTolerance(0.00001);
          //optimizer->SetRelaxationFactor( 0.8 );

          // Create the Command observer and register it with the optimizer.
          //
          CommandIterationUpdate::Pointer observer = CommandIterationUpdate::New();
          optimizer->AddObserver( itk::IterationEvent(), observer );

          // One level registration process without shrinking and smoothing.
          //
          const unsigned int numberOfLevels = 1;

          RegistrationType::ShrinkFactorsArrayType shrinkFactorsPerLevel;
          shrinkFactorsPerLevel.SetSize( 1 );
          shrinkFactorsPerLevel[0] = 3;

          RegistrationType::SmoothingSigmasArrayType smoothingSigmasPerLevel;
          smoothingSigmasPerLevel.SetSize( 1 );
          smoothingSigmasPerLevel[0] = 0;

          registration->SetNumberOfLevels ( numberOfLevels );
          registration->SetSmoothingSigmasPerLevel( smoothingSigmasPerLevel );
          registration->SetShrinkFactorsPerLevel( shrinkFactorsPerLevel );

          try
            {
            registration->Update();
            std::cout << "Optimizer stop condition: "
                      << registration->GetOptimizer()->GetStopConditionDescription()
                      << std::endl;
            }
          catch( itk::ExceptionObject & err )
            {
            std::cerr << "ExceptionObject caught !" << std::endl;
            std::cerr << err << std::endl;
            return EXIT_FAILURE;
            }

          TransformType::ParametersType finalParameters =
                                    registration->GetOutput()->Get()->GetParameters();

          unsigned long numberOfIterations = optimizer->GetCurrentIteration();

          // Catching angle and final x translation

          TransformType::Pointer finalTransform = TransformType::New();
          finalTransform->SetFixedParameters( registration->GetOutput()->Get()->GetFixedParameters() );
          finalTransform->SetParameters( finalParameters );

          TransformType::MatrixType matrix = finalTransform->GetMatrix();
          TransformType::OffsetType offset = finalTransform->GetOffset();
          std::cout << "Matrix = " << std::endl << matrix << std::endl;
          std::cout << "Offset = " << std::endl << offset << std::endl;

          // Calculate the average values for x and y components
          double angleDegree = std::asin(matrix[0][1]) * (180.0/3.141592653589793238463);

          meanNumberOfIterations += numberOfIterations;
          meanX += offset[0];
          meanY += offset[1];
          meanZ += offset[2];
          meanAngle += angleDegree;

          // Writing the transform file and displacement field only once.
          if (y == n_times - 1){

              std::ostringstream val;
              val << std::setprecision(2) << qValue;

              // Writing Transform
              using TransformWriterType = itk::TransformFileWriter;
              TransformWriterType::Pointer transformWriter = TransformWriterType::New();
              transformWriter->SetInput(finalTransform);
              transformWriter->SetFileName(imagePath + "resultTransform-q="+val.str()+".tfm");
              transformWriter->Update();

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

              TransformType::InputPointType movingPoint;
              TransformType::OutputPointType transformedPoint;
              DisplacementFieldType::IndexType index;

              VectorType displacement;

              while( !fi.IsAtEnd()){
                  index = fi.GetIndex();
                  field->TransformIndexToPhysicalPoint( index, movingPoint );
                  transformedPoint = finalTransform->TransformPoint( movingPoint );
                  displacement = transformedPoint - movingPoint;
                  fi.Set(displacement);
                  ++fi;
              }

              using FieldWriterType = itk::ImageFileWriter<DisplacementFieldType>;
              FieldWriterType::Pointer fieldWriter = FieldWriterType::New();
              fieldWriter->SetInput(field);
              fieldWriter->SetFileName(imagePath + "dispField-q=" + val.str() + ".nrrd");
              fieldWriter->Update();

          }

          // Destroying smart pointers to clean up processing memory
          optimizer.~SmartPointer();
          observer.~SmartPointer();
          registration.~SmartPointer();
          metric.~SmartPointer();
          finalTransform.~SmartPointer();
          observer.~SmartPointer();

        } // End of the average loop

      // Storing Average Results

      meanNumberOfIterations = meanNumberOfIterations/n_times;
      meanX = meanX/n_times;
      meanY = meanY/n_times;
      meanZ = meanZ/n_times;
      meanAngle = meanAngle/n_times;

      // Print out results
      //
      std::cout << std::endl;
      std::cout << " Result               = "   << std::endl;
      std::cout << " qValue               = "   << qValue <<" Execution: "<< i << std::endl;
      std::cout << " Iterations           = "   << meanNumberOfIterations << std::endl;
      std::cout << " X,Y, and Z,          = "   << meanX<<", "<< meanY<<", "<< meanZ << std::endl;
      std::cout << " Angle Relative  = "   << meanAngle << std::endl;
      std::cout << std::endl;

      myfile <<qValue<<","<<meanNumberOfIterations<<","<<meanX<<","<<meanY<<","<<meanZ<<","<<meanAngle<< std::endl;

  } // End of q-value loop

  myfile.close();

  return EXIT_SUCCESS;
}
