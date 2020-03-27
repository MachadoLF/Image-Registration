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

//  Software Info:
//  Q-value Searcher for 2D rigid problems
//  Using the WHOLE image intead of 20% sampling.

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
  int x_times = 5;

  myfile <<"# All the points presented here is averaged over "<<x_times<<" times execution."<<std::endl;
  myfile <<"qValue,Number of Iterations,RelativeError"<<std::endl;

  double qValueIni = -0.5;
  for (unsigned int i = 0; i < numOfInstances; ++i) // qValue Loop
  {
      double meanNumberOfIterations = 0.0;
      double meanRelativeError = 0.0;

      double qValue = qValueIni + 0.01*i;

      if (qValue == 1.0){
          continue;
      }

      for (int y = 0; y < x_times; ++y) { // Average x times loop

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

          double TranslationAlongX = finalParameters[0];
          double TranslationAlongY = finalParameters[1];

          unsigned long numberOfIterations = optimizer->GetCurrentIteration();

          // Catching execution parameter for further evaluations

          meanNumberOfIterations += numberOfIterations + 1.0;

          // Calculate the relative error considering the expected values for x and y components
          // Expression used for absolut relative error calculation
          meanRelativeError += (13.0 - std::abs(TranslationAlongX))/13.0 + (17.0 - std::abs(TranslationAlongY))/17.0;

          // Destrying smart pointers to clean up processing memory
          optimizer.~SmartPointer();
          observer.~SmartPointer();
          registration.~SmartPointer();
          metric.~SmartPointer();

        } // End of the average loop

      // Storing Average Results

      meanNumberOfIterations = meanNumberOfIterations/x_times;
      meanRelativeError = meanRelativeError/x_times;

      // Print out results
      //
      std::cout << std::endl;
      std::cout << " Results         = "   << std::endl;
      std::cout << " qValue         = "   << qValue <<" Execution: "<< i << std::endl;
      std::cout << " Iterations     = "   << meanNumberOfIterations << std::endl;
      std::cout << " Relative Error = "   << meanRelativeError << std::endl;

      std::cout << std::endl;

      myfile <<qValue<<","<<meanNumberOfIterations<<","<<meanRelativeError<< std::endl;

  } // End of q-value loop

  myfile.close();

  return EXIT_SUCCESS;
}
