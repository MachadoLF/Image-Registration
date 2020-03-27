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
#include "itkMachadoMutualInformationImageToImageMetricv4.h"
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

    myfile <<optimizer->GetCurrentIteration()+1<<","<<optimizer->GetValue()<<std::endl;
    }
};


int main( int argc, char *argv[] )
{
  if( argc < 2 )
    {
    std::cerr << "Missing Parameters " << std::endl;
    std::cerr << "Usage: " << argv[0];
    std::cerr << " fixedImageFile  movingImageFile ";
    std::cerr << "[qValue] [xTimes to average]";
    std::cerr << "[useExplicitPDFderivatives ] " << std::endl;
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

  // registration pointer for passage;
  RegistrationType::Pointer registrationPass;

  // Defining specific metric
  // Tsallis
  // typedef itk::MachadoMutualInformationImageToImageMetricv4< FixedImageType,MovingImageType > MetricType;

  // Mattes
  typedef itk::MattesMutualInformationImageToImageMetricv4< FixedImageType,MovingImageType > MetricType;
  // Software Guide : EndCodeSnippet


  typedef itk::ImageFileReader< FixedImageType  > FixedImageReaderType;
  typedef itk::ImageFileReader< MovingImageType > MovingImageReaderType;

  FixedImageReaderType::Pointer  fixedImageReader  = FixedImageReaderType::New();
  MovingImageReaderType::Pointer movingImageReader = MovingImageReaderType::New();

  fixedImageReader->SetFileName(  argv[1] );
  movingImageReader->SetFileName( argv[2] );


  // itinitiallizing qValue and average times

  double qValue = 0.5;
  if( argc > 2 )
  {
      qValue = atof(argv[3]);
  }

  int xTimes = 1;
  if( argc > 3 )
  {
      xTimes = atoi(argv[4]);
  }

  double meanMetricValue = 0.0;
  double meanNumberOfIterations = 0.0;
  double meanXError = 0.0;
  double meanYError = 0.0;


  for (int y = 0; y < xTimes; ++y){

      myfile.open ("performance.csv");
      myfile <<"# The performance was carried with qValue = "<<qValue<<" . Each point is equal to a "<<xTimes<<" performance execution."<< std::endl;
      myfile <<"Iteration,Relative Error"<<std::endl;

      RegistrationType::Pointer   registration  = RegistrationType::New();
      registrationPass = registration;

      OptimizerType::Pointer       optimizer    = OptimizerType::New();

      registration->SetOptimizer(     optimizer     );
      // Setting images

      // Setting Metric
      MetricType::Pointer metric = MetricType::New();
      // metric->SetqValue(qValue);
      registration->SetMetric( metric  );

      //  The metric requires the user to specify the number of bins
      //  used to compute the entropy. In a typical application, 50 histogram bins
      //  are sufficient.

      unsigned int numberOfBins = 24;

      /*
      if( argc > 7 )
        {
        numberOfBins = atoi( argv[7] );
        }
      */
      metric->SetNumberOfHistogramBins( numberOfBins );

      metric->SetUseMovingImageGradientFilter( false );
      metric->SetUseFixedImageGradientFilter( false );

      // It will use the whole image to test.
      metric->SetUseSampledPointSet(false);

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

      // Instead of using the whole virtual domain (usually fixed image domain) for the registration,
      // we can use a spatial sampled point set by supplying an arbitrary point list over which to
      // evaluate the metric.

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

      double TranslationAlongX = finalParameters[0];
      double TranslationAlongY = finalParameters[1];

      // For stability reasons it may be desirable to round up the values of translation
      //
      unsigned int numberOfIterations = optimizer->GetCurrentIteration();
                                        // -1.0 is used for correcting the number of executions.
      double metricValue = optimizer->GetCurrentMetricValue();

      // Expression used for absolut relative error calculation
      double xError = std::abs(13.0 - TranslationAlongX);
      double yError = std::abs(17.0 - TranslationAlongY);

      meanNumberOfIterations += numberOfIterations;
      meanXError += xError;
      meanYError += yError;
      meanMetricValue += metricValue;

      myfile.close();

      // condition for destroying smartPointers only
      // when new executions will come afterwards
      if (y < xTimes - 1){
              optimizer.~SmartPointer();
              observer.~SmartPointer();
              registration.~SmartPointer();
              metric.~SmartPointer();
      }
   }

  // Printing out results
  meanXError = meanXError/xTimes;
  meanYError = meanYError/xTimes;
  meanNumberOfIterations = meanNumberOfIterations/xTimes;
  meanMetricValue = meanMetricValue/xTimes;

  double meanTotalError = meanXError + meanYError;

  std::cout << std::endl;
  std::cout << " Result          = " << std::endl;
  std::cout << " qValue          = " << qValue << std::endl;
  std::cout << " Iterations      = " << meanNumberOfIterations<< std::endl;
  std::cout << " meanXError      = " << meanXError<< std::endl;
  std::cout << " meanYError      = " << meanYError<< std::endl;
  std::cout << " Total Error  = " << meanTotalError << std::endl;
  std::cout << " Metric Value  = " << meanMetricValue << std::endl;
  std::cout << std::endl;

  //  \item \code{BrainT1SliceBorder20.png}
  //  \item \code{BrainProtonDensitySliceShifted13x17y.png}

  //  \code{Brain\-Proton\-Density\-Slice\-Border20.png} by $(13,17)$
  //  millimeters. Both images have unit-spacing and are shown in Figure
  //  \ref{fig:FixedMovingImageRegistration2}. The registration process
  //  converges after $46$ iterations and produces the following results:

  //  Translation X = 13.0204
  //  Translation Y = 17.0006

  // Writing OUTPUT images

  typedef itk::ResampleImageFilter<
                            MovingImageType,
                            FixedImageType >    ResampleFilterType;

  ResampleFilterType::Pointer resample = ResampleFilterType::New();

  resample->SetTransform( registrationPass->GetTransform() );
  resample->SetInput( movingImageReader->GetOutput() );

  FixedImageType::Pointer fixedImage = fixedImageReader->GetOutput();

  PixelType defaultPixelValue = 100;

  // Seting aditional resampling information.
  resample->SetSize(  fixedImage->GetLargestPossibleRegion().GetSize() );
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

  writer->SetFileName( "OutPut.png" );

  caster->SetInput( resample->GetOutput() );
  writer->SetInput( caster->GetOutput()   );
  writer->Update();


  // Generate checkerboards before and after registration
  //
  typedef itk::CheckerBoardImageFilter< FixedImageType > CheckerBoardFilterType;

  CheckerBoardFilterType::Pointer checker = CheckerBoardFilterType::New();

  // Inputting the fied image and the resampler filter
  // inputted with the moving image.
  // By changing the transform, one can export before or after
  // registration results.

  checker->SetInput1( fixedImage );
  checker->SetInput2( resample->GetOutput() );

  caster->SetInput( checker->GetOutput() );
  writer->SetInput( caster->GetOutput()  );

  resample->SetDefaultPixelValue( 0 );

  // Before registration ================
  // It will set the identity transform to the moving image
  // resampling at - resample filter - .

  TransformType::Pointer identityTransform = TransformType::New();
  identityTransform->SetIdentity();
  resample->SetTransform( identityTransform );

  writer->SetFileName( "CheckBoardBefore.png" );
  writer->Update();

  // After registration =================
  // Set the last transformation obtainned in the registrations executions

  resample->SetTransform( registrationPass->GetTransform() );
  writer->SetFileName( "CheckBoardAfter.png" );
  writer->Update();

  return EXIT_SUCCESS;
}
