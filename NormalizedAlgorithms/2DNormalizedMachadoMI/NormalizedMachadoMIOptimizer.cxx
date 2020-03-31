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
//    OUTPUTS: {ImageRegistration4Output.png}
//    ARGUMENTS:    100
//    OUTPUTS: {ImageRegistration4CheckerboardBefore.png}
//    OUTPUTS: {ImageRegistration4CheckerboardAfter.png}
//    ARGUMENTS:    24
//  Software Guide : EndCommandLineArgs

// Software Guide : BeginLatex
//
// In this example, we will solve a simple multi-modality problem using an
// implementation of mutual information. This implementation was published by
// Mattes~\emph{et. al}~\cite{Mattes2003}.
//
// First, we include the header files of the components used in this example.
//
// \index{itk::ImageRegistrationMethodv4!Multi-Modality}
//
// Software Guide : EndLatex


// Software Guide : BeginCodeSnippet
#include "itkImageRegistrationMethodv4.h"
#include "itkTranslationTransform.h"
#include "itkNormalizedMachadoMutualInformationImageToImageMetricv4.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"
// Software Guide : EndCodeSnippet

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
  itkNewMacro( Self );

protected:
  CommandIterationUpdate() {};

public:
  typedef itk::RegularStepGradientDescentOptimizerv4<double> OptimizerType;
  typedef   const OptimizerType *                            OptimizerPointer;

  void Execute(itk::Object *caller, const itk::EventObject & event) ITK_OVERRIDE
    {
    Execute( (const itk::Object *)caller, event);
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

  const    unsigned int    Dimension = 2;
  typedef  float           PixelType;

  typedef itk::Image< PixelType, Dimension >  FixedImageType;
  typedef itk::Image< PixelType, Dimension >  MovingImageType;

  typedef itk::TranslationTransform< double, Dimension >         TransformType;
  typedef itk::RegularStepGradientDescentOptimizerv4<double>     OptimizerType;
  typedef itk::ImageRegistrationMethodv4<
                                    FixedImageType,
                                    MovingImageType,
                                    TransformType    > RegistrationType;


  myfile.open ("performance.csv");

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

  RegistrationType::Pointer   registration  = RegistrationType::New();

  OptimizerType::Pointer      optimizer     = OptimizerType::New();

  // Number of qValues tested
  unsigned int numOfInstances = 1;
  int x_times = 1;

  myfile <<"# All the points presented here is averaged over "<<x_times<<" times execution."<<std::endl;
  myfile <<"qValue,Elapsed Time,Number of Iterations,Metric Value,RelativeError"<<std::endl;

  double qValueIni = 0.8;
  for (unsigned int i = 0; i < numOfInstances; ++i) // qValue Loop
  {
      double meanElapsedTime = 0.0;
      double meanNumberOfIterations = 0.0;
      double meanMetricValue = 0.0;

      double meanRelativeError = 0.0;

      double qValue = qValueIni + 0.01*i;


      if (qValue == 1.0){
          continue;
      }

      for (int y = 0; y < x_times; ++y) { // Average x times loop

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
            numberOfBins = atoi( argv[3] );
            }

          metric->SetNumberOfHistogramBins( numberOfBins );

          metric->SetUseMovingImageGradientFilter( false );
          metric->SetUseFixedImageGradientFilter( false );

          // Using the whole image in the Mutual information calculation
          metric->SetUseSampledPointSet( false );

          registration->SetFixedImage(    fixedImageReader->GetOutput()    );
          registration->SetMovingImage(   movingImageReader->GetOutput()   );


          // Cinfiguring the optimizer

          optimizer->SetLearningRate( 8.00 );
          optimizer->SetMinimumStepLength( 0.0001 );
          optimizer->SetNumberOfIterations( 200 );
          optimizer->ReturnBestParametersAndValueOn();
          optimizer->SetGradientMagnitudeTolerance(0.00001);

          optimizer->SetRelaxationFactor( 0.8 );

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

          /*
          RegistrationType::MetricSamplingStrategyType  samplingStrategy  =
            RegistrationType::RANDOM;

          double samplingPercentage = 0.20;

          // In ITKv4, a single virtual domain or spatial sample point set is used for the
          // all iterations of the registration process. The use of a single sample set results
          // in a smooth cost function that can improve the functionality of
          // the optimizer.
          //
          // The spatial point set is pseudo randomly generated. For
          // reproducible results an integer seed should set.


          registration->SetMetricSamplingStrategy( samplingStrategy );
          registration->SetMetricSamplingPercentage( samplingPercentage );


          registration->MetricSamplingReinitializeSeed( 121213 );
          */

          auto begin = std::chrono::high_resolution_clock::now();
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
          auto end = std::chrono::high_resolution_clock::now();


          TransformType::ParametersType finalParameters =
                                    registration->GetOutput()->Get()->GetParameters();

          double TranslationAlongX = finalParameters[0];
          double TranslationAlongY = finalParameters[1];

          unsigned long numberOfIterations = optimizer->GetCurrentIteration();

          double bestValue = optimizer->GetValue();

          double elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

          std::cout << std::endl;
          std::cout << " Result         = "   << std::endl;
          std::cout << " qValue         = "   << qValue <<" Execution: "<< i << std::endl;
          std::cout << " Iterations     = "   << numberOfIterations << std::endl;
          std::cout << " Metric value   = "   << bestValue << std::endl;
          std::cout << " Elapsed Time   = "   << elapsedTime << "[ms]" << std::endl;
          //std::cout << " Relative Error = "   << meanRelativeError << std::endl;

          /* Catching execution parameter for further evaluations

          meanMetricValue += bestValue;
          meanElapsedTime += elapsedTime;
          meanNumberOfIterations += numberOfIterations;

          // Calculate the relative error considering the expected values for x and y components

          meanRelativeError += std::abs((13.0 - TranslationAlongX)/13.0) + std::abs((17.0 - TranslationAlongY)/17.0);

          // Destrying smart pointers to clean up processing memory

          optimizer.~SmartPointer();
          observer.~SmartPointer();
          registration.~SmartPointer();
          metric.~SmartPointer();
          */

        } // End of the average loop

      // Storing Average Results

      //meanElapsedTime = meanElapsedTime/x_times;
      //meanMetricValue = meanMetricValue/x_times;
      //meanNumberOfIterations = meanNumberOfIterations/x_times;
      //meanRelativeError = meanRelativeError/x_times;

      // Print out results
      //

      std::cout << std::endl;

      //myfile <<qValue<<","<<meanElapsedTime<<","<<meanNumberOfIterations<<","<<meanMetricValue<<","<<meanRelativeError<< std::endl;

  } // End of q-value loop

  myfile.close();

  typedef itk::ResampleImageFilter<
                              MovingImageType,
                              FixedImageType >    ResampleFilterType;

    ResampleFilterType::Pointer resample = ResampleFilterType::New();

    resample->SetTransform( registration->GetTransform() );
    resample->SetInput( movingImageReader->GetOutput() );

    FixedImageType::Pointer fixedImage = fixedImageReader->GetOutput();

    PixelType defaultPixelValue = 100;

    if( argc > 4 )
      {
      defaultPixelValue = atoi( argv[4] );
      }

    resample->SetSize(    fixedImage->GetLargestPossibleRegion().GetSize() );
    resample->SetOutputOrigin(  fixedImage->GetOrigin() );
    resample->SetOutputSpacing( fixedImage->GetSpacing() );
    resample->SetOutputDirection( fixedImage->GetDirection() );
    resample->SetDefaultPixelValue( defaultPixelValue );


    typedef  unsigned char  OutputPixelType;

    typedef itk::Image< OutputPixelType, Dimension > OutputImageType;

    typedef itk::CastImageFilter<
                          FixedImageType,
                          OutputImageType > CastFilterType;

    typedef itk::ImageFileWriter< OutputImageType >  WriterType;

    WriterType::Pointer      writer =  WriterType::New();
    CastFilterType::Pointer  caster =  CastFilterType::New();

    writer->SetFileName( "result.jpg" );

    caster->SetInput( resample->GetOutput() );
    writer->SetInput( caster->GetOutput()   );
    writer->Update();

    //
    // Generate checkerboards before and after registration
    //
    typedef itk::CheckerBoardImageFilter< FixedImageType > CheckerBoardFilterType;

    CheckerBoardFilterType::Pointer checker = CheckerBoardFilterType::New();

    checker->SetInput1( fixedImage );
    checker->SetInput2( resample->GetOutput() );

    caster->SetInput( checker->GetOutput() );
    writer->SetInput( caster->GetOutput()   );

    resample->SetDefaultPixelValue( 0 );

    // Before registration
    TransformType::Pointer identityTransform = TransformType::New();
    identityTransform->SetIdentity();

    resample->SetTransform( identityTransform );
    writer->SetFileName( "before.png" );
    writer->Update();


    // After registration
    resample->SetTransform( registration->GetTransform() );
    writer->SetFileName( "after.png" );
    writer->Update();

  return EXIT_SUCCESS;
}
