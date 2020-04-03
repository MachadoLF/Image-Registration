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

    myfile <<optimizer->GetCurrentIteration()<<","<<optimizer->GetValue()<<std::endl;
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

  double qValue = 0.9;

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

  // double TranslationAlongX = finalParameters[0];
  // double TranslationAlongY = finalParameters[1];

  unsigned long numberOfIterations = optimizer->GetCurrentIteration();

  double bestValue = optimizer->GetValue();

  std::cout << std::endl;
  std::cout << " Result         = "   << std::endl;
  std::cout << " qValue         = "   << qValue << std::endl;
  std::cout << " Iterations     = "   << numberOfIterations << std::endl;
  std::cout << " Metric value   = "   << bestValue << std::endl;
  //std::cout << " Relative Error = "   << meanRelativeError << std::endl;

  std::cout << std::endl;

  //myfile <<qValue<<","<<meanElapsedTime<<","<<meanNumberOfIterations<<","<<meanMetricValue<<","<<meanRelativeError<< std::endl;

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
