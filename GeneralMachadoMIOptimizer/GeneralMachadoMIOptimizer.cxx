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

#include "itkImageRegistrationMethodv4.h"
#include "itkTranslationTransform.h"
#include "itkNormalizedMachadoMutualInformationImageToImageMetricv4.h"
#include "itkMachadoMutualInformationImageToImageMetricv4.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"
#include "itkOnePlusOneEvolutionaryOptimizerv4.h"
#include "itkLBFGSOptimizerv4.h"

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
#include "itkResampleImageFilter.h"

#include "iomanip"


//  using myFileType = std::ofstream;
//  static myFileType myfile;

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
    typedef itk::LBFGSOptimizerv4 OptimizerType;
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

    using VectorType = itk::Vector<float, Dimension>;
    using DisplacementFieldType = itk::Image<VectorType, Dimension>;
    using FieldWriterType = itk::ImageFileWriter <DisplacementFieldType>;
    // using FieldArray = itk::Vector<DisplacementFieldType::Pointer>;

    // typedef itk::TranslationTransform< double, Dimension >         TransformType;
    using TransformType = itk::VersorRigid3DTransform< double >;
    typedef itk::LBFGSOptimizerv4                   OptimizerType;

    // using OptimizerTypeOPO = itk::OnePlusOneEvolutionaryOptimizerv4<double>;

    typedef itk::ImageRegistrationMethodv4<
            FixedImageType,
            MovingImageType,
            TransformType    >                  RegistrationType;

    // std::cerr << "Chegou Aqui! " << std::endl;
    std::string imagePath = argv[3];
    // myfile.open (imagePath + "performance.csv");


    typedef itk::ImageFileReader< FixedImageType  > FixedImageReaderType;
    typedef itk::ImageFileReader< MovingImageType > MovingImageReaderType;

    FixedImageReaderType::Pointer  fixedImageReader  = FixedImageReaderType::New();
    MovingImageReaderType::Pointer movingImageReader = MovingImageReaderType::New();

    fixedImageReader->SetFileName( argv[1] );
    movingImageReader->SetFileName( argv[2] );

    // Defining specific metric
    unsigned int numberOfBins = 50;

    // Normalized Tsallis MI Metric
    typedef itk::NormalizedMachadoMutualInformationImageToImageMetricv4< FixedImageType,MovingImageType > NormTMIMetricType;
    NormTMIMetricType::Pointer normTmiMetric = NormTMIMetricType::New();
    normTmiMetric->SetNumberOfHistogramBins( numberOfBins );
    normTmiMetric->SetUseMovingImageGradientFilter( false );
    normTmiMetric->SetUseFixedImageGradientFilter( false );
    normTmiMetric->SetUseSampledPointSet( false );

    // Normalized Tsallis MI Metric
    typedef itk::MachadoMutualInformationImageToImageMetricv4< FixedImageType,MovingImageType > TMIMetricType;
    TMIMetricType::Pointer tmiMetric = TMIMetricType::New();
    tmiMetric->SetNumberOfHistogramBins( numberOfBins );
    tmiMetric->SetUseMovingImageGradientFilter( false );
    tmiMetric->SetUseFixedImageGradientFilter( false );
    tmiMetric->SetUseSampledPointSet( false );

    // Mattes
    typedef itk::MattesMutualInformationImageToImageMetricv4< FixedImageType,MovingImageType > SMIMetricType;
    SMIMetricType::Pointer smiMetric = SMIMetricType::New();
    smiMetric->SetNumberOfHistogramBins( numberOfBins );
    smiMetric->SetUseMovingImageGradientFilter( false );
    smiMetric->SetUseFixedImageGradientFilter( false );
    smiMetric->SetUseSampledPointSet( false );

    // Number of qValues tested
    unsigned int numOfInstances = 10;
    double qValueIni = 0.1;

    for (unsigned int i = 0; i < numOfInstances; ++i) // qValue Loop
    {
        double meanNumberOfIterations = 0.0;

        double qValue = qValueIni + i*0.1;
        std::cout << "q-value = " << qValue << std::endl;

        // setting q-value for both nonNorm and Norm metric
        tmiMetric->SetqValue(qValue);
        normTmiMetric->SetqValue(qValue);

        // Metric selector loop - For each q-value, it will execute a metric test.
        for (unsigned int m = 0; m < 2; ++m) {
            std::string metricId;
            RegistrationType::Pointer   registration  = RegistrationType::New();
            OptimizerType::Pointer      optimizer     = OptimizerType::New();

            registration->SetOptimizer(     optimizer     );

            // Metric selection
            if (qValue == 1.0){
                registration->SetMetric( smiMetric );
                std::string metricType = "Shannon MI metric.";
                //metricId = "shannon";

                // Parameter scale setter
                using ParameterEstimatorType = itk::RegistrationParameterScalesFromPhysicalShift< SMIMetricType >;
                ParameterEstimatorType::Pointer estimator = ParameterEstimatorType::New();
                estimator->SetMetric(smiMetric);
                estimator->SetTransformForward(true);
                // optimizer->SetScalesEstimator(estimator);

            } else if ( m == 0) {
                registration->SetMetric( tmiMetric );
                std::string metricType = "Non Norm. TMI metric.";
                metricId = "NonNormalized-LBFGS";

                // Parameter scale setter
                using ParameterEstimatorType = itk::RegistrationParameterScalesFromPhysicalShift< TMIMetricType >;
                ParameterEstimatorType::Pointer estimator = ParameterEstimatorType::New();
                estimator->SetMetric(tmiMetric);
                estimator->SetTransformForward(true);
                // optimizer->SetScalesEstimator(estimator);

            } else if ( m == 1) {
                registration->SetMetric( normTmiMetric );
                std::string metricType = "Norm. TMI metric.";
                metricId = "Normalized-LBFGS";

                // Parameter scale setter
                using ParameterEstimatorType = itk::RegistrationParameterScalesFromPhysicalShift< NormTMIMetricType >;
                ParameterEstimatorType::Pointer estimator = ParameterEstimatorType::New();
                estimator->SetMetric(normTmiMetric);
                estimator->SetTransformForward(true);
                // optimizer->SetScalesEstimator(estimator);
            }

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
            VersorType rotation;
            VectorType axis;
            axis[0] = 0.0;
            axis[1] = 0.0;
            axis[2] = 1.0;
            constexpr double angle = 0;
            rotation.Set(  axis, angle  );
            initialTransform->SetRotation( rotation );

            registration->SetInitialTransform( initialTransform );

            // Configuring the optimizer

            optimizer->SetGradientConvergenceTolerance(0.05);
            optimizer->SetLineSearchAccuracy(0.9);
            optimizer->SetDefaultStepLength(.5);
            optimizer->TraceOn();
            optimizer->SetMaximumNumberOfFunctionEvaluations(1000);
            //optimizer->SetNumberOfIterations( 300 );

            /*
            optimizer->SetDoEstimateLearningRateOnce(false);
            optimizer->SetDoEstimateLearningRateAtEachIteration(false);

            optimizer->SetLearningRate( 0.2 );
            optimizer->SetMinimumStepLength( 0.001 );
            optimizer->ReturnBestParametersAndValueOn();
            optimizer->SetGradientMagnitudeTolerance(0.00001);

            // Let's avoid setting Tolerance and RelaxationFactor
            //optimizer->SetRelaxationFactor( 0.8 );
            */
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

            // Catching angle and final transform
            TransformType::Pointer finalTransform = TransformType::New();
            finalTransform->SetFixedParameters( registration->GetOutput()->Get()->GetFixedParameters() );
            finalTransform->SetParameters( finalParameters );

            std::ostringstream val;
            val << std::setprecision(3) << qValue;
            std::string id = val.str();

            // Writing Transform
            using TransformWriterType = itk::TransformFileWriter;
            TransformWriterType::Pointer transformWriter = TransformWriterType::New();
            transformWriter->SetInput(finalTransform);
            transformWriter->SetFileName(imagePath + "resultTransform-q="+id+".tfm");
            transformWriter->Update();

            // Creating and writing the displacement vector image
            // Based on example: https://itk.org/Doxygen/html/Examples_2RegistrationITKv3_2DeformableRegistration8_8cxx-example.html

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
                transformedPoint = finalTransform->TransformPoint( movingPoint );
                displacement = transformedPoint - movingPoint;
                fi.Set(displacement);
                ++fi;
            }

            using ResamplerType = itk::ResampleImageFilter<DisplacementFieldType, DisplacementFieldType>;
            ResamplerType::Pointer resampler = ResamplerType::New();
            resampler->SetInput(field);
            resampler->SetSize(fixedImageReader->GetOutput()->GetLargestPossibleRegion().GetSize());
            resampler->SetOutputOrigin(fixedImageReader->GetOutput()->GetOrigin());
            resampler->SetOutputSpacing(fixedImageReader->GetOutput()->GetSpacing());
            resampler->SetOutputDirection(fixedImageReader->GetOutput()->GetDirection());
            resampler->Update();

            FieldWriterType::Pointer fieldWriter = FieldWriterType::New();
            fieldWriter->SetInput(resampler->GetOutput());
            fieldWriter->SetFileName(imagePath + "dispField-q=" + id + ".nrrd");
            fieldWriter->Update();

            // Destroying smart pointers to clean up processing memory
            optimizer.~SmartPointer();
            registration.~SmartPointer();

            // Print out results
            //
            std::cout << std::endl;
            std::cout << " Result               = "   << std::endl;
            std::cout << " qValue               = "   << qValue <<" Execution: "<< i << std::endl;
            std::cout << " Iterations           = "   << meanNumberOfIterations << std::endl;
            std::cout << std::endl;

        } // Endo of metric loops

    } // End of q-value loop

    return EXIT_SUCCESS;
}
