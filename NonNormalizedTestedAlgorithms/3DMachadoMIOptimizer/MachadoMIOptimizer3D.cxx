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
//    INPUTS:  {BrainT1SliceBorder20.png}
//    INPUTS:  {BrainProtonDensitySliceShifted13x17y.png}
//    OUTPUTS: {performance.csv}

//  This algorithm intends to find the optmum q-value
//  for a 3-D rigid multy-modality registration

// Software Guide : BeginCodeSnippet
#include "itkImageRegistrationMethodv4.h"
#include "itkTranslationTransform.h"
#include "itkMachadoMutualInformationImageToImageMetricv4.h"
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

#include "chrono"

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
  if( argc < 3 )
    {
    std::cerr << "Missing Parameters " << std::endl;
    std::cerr << "Usage: " << argv[0];
    std::cerr << " fixedImageFile  movingImageFile "<< std::endl;
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


  myfile.open ("performance.csv");

  // Defining specific metric
  // Tsallis
  typedef itk::MachadoMutualInformationImageToImageMetricv4< FixedImageType,MovingImageType > MetricType;

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
  unsigned int numOfInstances = 200;
  int n_times = 3;

  myfile <<"# All the points presented here are averaged over "<<n_times<<" times execution."<<std::endl;
  myfile <<"qValue,Number of Iterations,XError,YError,ZError,AngleError,BuildupError"<<std::endl;

  double qValueIni = -0.5;
  for (unsigned int i = 0; i < numOfInstances; ++i) // qValue Loop
  {
      double meanNumberOfIterations = 0.0;
      double meanXError = 0.0;
      double meanYError = 0.0;
      double meanZError = 0.0;
      double meanAngleError = 0.0;
      double meanBuildupError = 0.0;

      double qValue = qValueIni + 0.01*i;

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

          if( argc > 3 )
            {
            numberOfBins = static_cast<unsigned int>( atoi( argv[3] ) );
            }

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

          initializer->MomentsOn();

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

          using OptimizerScalesType = OptimizerType::ScalesType;
          OptimizerScalesType optimizerScales( initialTransform->GetNumberOfParameters() );
          const double translationScale = 1.0 / 1000.0;
          optimizerScales[0] = 1.0;
          optimizerScales[1] = 1.0;
          optimizerScales[2] = 1.0;
          optimizerScales[3] = translationScale;
          optimizerScales[4] = translationScale;
          optimizerScales[5] = translationScale;
          optimizer->SetScales( optimizerScales );

          // Configuring the optimizer

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
          shrinkFactorsPerLevel[0] = 1;

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

          //const double versorX              = finalParameters[0];
          //const double versorY              = finalParameters[1];
          //const double versorZ              = finalParameters[2];
          //const double finalTranslationX    = finalParameters[3];
          //const double finalTranslationY    = finalParameters[4];
          //const double finalTranslationZ    = finalParameters[5];
          unsigned long numberOfIterations = optimizer->GetCurrentIteration();

          meanNumberOfIterations += numberOfIterations;

          // Catching angle and final x translation

          TransformType::Pointer finalTransform = TransformType::New();
          finalTransform->SetFixedParameters( registration->GetOutput()->Get()->GetFixedParameters() );
          finalTransform->SetParameters( finalParameters );

          TransformType::MatrixType matrix = finalTransform->GetMatrix();
          TransformType::OffsetType offset = finalTransform->GetOffset();
          std::cout << "Matrix = " << std::endl << matrix << std::endl;
          std::cout << "Offset = " << std::endl << offset << std::endl;
          //std::cout << "Matrix[0][1] = " << matrix[0][1] << std::endl;


          // Calculate the relative error considering the expected values for x and y components
          double angleDegree = std::asin(matrix[0][1]) * (180.0/3.141592653589793238463);

          std::cout << "Angle = " << angleDegree << std::endl;

          meanXError += std::abs( -15.0 - offset[0] );
          meanYError += std::abs( 0.0001 - offset[1] );
          meanZError += std::abs( 0.0001 - offset[2] );
          meanAngleError += std::abs(10.0 - angleDegree);

          // Destroying smart pointers to clean up processing memory
          optimizer.~SmartPointer();
          observer.~SmartPointer();
          registration.~SmartPointer();
          metric.~SmartPointer();

        } // End of the average loop

      // Storing Average Results

      meanNumberOfIterations = meanNumberOfIterations/n_times;
      meanXError = meanXError/n_times;
      meanYError = meanYError/n_times;
      meanZError = meanZError/n_times;
      meanAngleError = meanAngleError/n_times;
      meanBuildupError = meanXError/(15.0) + meanYError/0.0001 + meanZError/0.0001 + meanAngleError/10.0;

      // Print out results
      //
      std::cout << std::endl;
      std::cout << " Result               = "   << std::endl;
      std::cout << " qValue               = "   << qValue <<" Execution: "<< i << std::endl;
      std::cout << " Iterations           = "   << meanNumberOfIterations << std::endl;
      std::cout << " X Relative Error     = "   << meanXError << std::endl;
      std::cout << " Angle Relative Error = "   << meanAngleError << std::endl;
      std::cout << " Buildup Relative Error = "   << meanBuildupError << std::endl;
      std::cout << std::endl;

      myfile <<qValue<<","<<meanNumberOfIterations<<","<<meanXError<<","<<meanYError<<","<<meanZError<<","<<meanAngleError<<","<<meanBuildupError<< std::endl;

  } // End of q-value loop

  myfile.close();

  return EXIT_SUCCESS;
}
