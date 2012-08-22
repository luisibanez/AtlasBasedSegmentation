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

#include "itkImageRegistrationMethod.h"
#include "itkMattesMutualInformationImageToImageMetric.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkRegularStepGradientDescentOptimizer.h"
#include "itkCenteredTransformInitializer.h"
#include "itkAffineTransform.h"
#include "itkImage.h"

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

#include "itkResampleImageFilter.h"
#include "itkCheckerBoardImageFilter.h"

//
//  The following piece of code implements an observer
//  that will monitor the evolution of the registration process.
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
  typedef itk::RegularStepGradientDescentOptimizer OptimizerType;
  typedef   const OptimizerType *                  OptimizerPointer;

  void Execute(itk::Object *caller, const itk::EventObject & event)
    {
    Execute( (const itk::Object *)caller, event);
    }

  void Execute(const itk::Object * object, const itk::EventObject & event)
    {
    OptimizerPointer optimizer =
                      dynamic_cast< OptimizerPointer >( object );
    if( ! itk::IterationEvent().CheckEvent( &event ) )
      {
      return;
      }
      std::cout << optimizer->GetCurrentIteration() << "   ";
      std::cout << optimizer->GetValue() << "   ";
      std::cout << optimizer->GetCurrentPosition() << std::endl;
    }
};


int main( int argc, char *argv[] )
{

  if( argc < 4 )
    {
    std::cerr << "Missing Parameters " << std::endl;
    std::cerr << "Usage: " << argv[0];
    std::cerr << "   fixedImageFile  movingImageFile " << std::endl;
    std::cerr << "   outputImagefile  [differenceBeforeRegistration] " << std::endl;
    std::cerr << "   [differenceAfterRegistration] " << std::endl;
    std::cerr << "   [stepLength] [maxNumberOfIterations] "<< std::endl;
    return EXIT_FAILURE;
    }


  const    unsigned int    Dimension = 3;
  typedef  float           PixelType;

  typedef itk::Image< PixelType, Dimension >  FixedImageType;
  typedef itk::Image< PixelType, Dimension >  MovingImageType;


  typedef itk::AffineTransform<
                                  double,
                                  Dimension  >     TransformType;


  typedef itk::RegularStepGradientDescentOptimizer       OptimizerType;

  typedef itk::MattesMutualInformationImageToImageMetric<
                                    FixedImageType,
                                    MovingImageType >    MetricType;

  typedef itk:: LinearInterpolateImageFunction<
                                    MovingImageType,
                                    double          >    InterpolatorType;

  typedef itk::ImageRegistrationMethod<
                                    FixedImageType,
                                    MovingImageType >    RegistrationType;


  MetricType::Pointer         metric        = MetricType::New();
  OptimizerType::Pointer      optimizer     = OptimizerType::New();
  InterpolatorType::Pointer   interpolator  = InterpolatorType::New();
  RegistrationType::Pointer   registration  = RegistrationType::New();

  registration->SetMetric(        metric        );
  registration->SetOptimizer(     optimizer     );
  registration->SetInterpolator(  interpolator  );


  metric->SetNumberOfHistogramBins( 50 );
  metric->ReinitializeSeed( 76926294 );
  metric->SetUseExplicitPDFDerivatives( true );
  metric->SetUseCachingOfBSplineWeights( true );
  metric->SetNumberOfSpatialSamples( 50000L );


  TransformType::Pointer  transform = TransformType::New();
  registration->SetTransform( transform );


  typedef itk::ImageFileReader< FixedImageType  > FixedImageReaderType;
  typedef itk::ImageFileReader< MovingImageType > MovingImageReaderType;

  FixedImageReaderType::Pointer  fixedImageReader  = FixedImageReaderType::New();
  MovingImageReaderType::Pointer movingImageReader = MovingImageReaderType::New();

  fixedImageReader->SetFileName(  argv[1] );
  movingImageReader->SetFileName( argv[2] );

  registration->SetFixedImage(    fixedImageReader->GetOutput()    );
  registration->SetMovingImage(   movingImageReader->GetOutput()   );

  fixedImageReader->Update();

  registration->SetFixedImageRegion(
     fixedImageReader->GetOutput()->GetBufferedRegion() );



  typedef itk::CenteredTransformInitializer<
                                    TransformType,
                                    FixedImageType,
                                    MovingImageType >  TransformInitializerType;

  TransformInitializerType::Pointer initializer = TransformInitializerType::New();

  TransformType::Pointer initialTransform = TransformType::New();

  initializer->SetTransform(   initialTransform );
  initializer->SetFixedImage(  fixedImageReader->GetOutput() );
  initializer->SetMovingImage( movingImageReader->GetOutput() );
  initializer->MomentsOn();
  initializer->InitializeTransform();

  transform->SetFixedParameters( initialTransform->GetFixedParameters() );

  registration->SetInitialTransformParameters( initialTransform->GetParameters() );


  double translationScale = 1.0 / 1000.0;
  if( argc > 8 )
    {
    translationScale = atof( argv[8] );
    }


  typedef OptimizerType::ScalesType       OptimizerScalesType;
  OptimizerScalesType optimizerScales( transform->GetNumberOfParameters() );

  optimizerScales[0] =  1.0;
  optimizerScales[1] =  1.0;
  optimizerScales[2] =  1.0;
  optimizerScales[3] =  1.0;
  optimizerScales[4] =  1.0;
  optimizerScales[5] =  1.0;
  optimizerScales[6] =  1.0;
  optimizerScales[7] =  1.0;
  optimizerScales[8] =  1.0;
  optimizerScales[9]  =  translationScale;
  optimizerScales[10] =  translationScale;
  optimizerScales[11] =  translationScale;

  optimizer->SetScales( optimizerScales );


  double steplength = 0.1;

  if( argc > 6 )
    {
    steplength = atof( argv[6] );
    }


  unsigned int maxNumberOfIterations = 300;

  if( argc > 7 )
    {
    maxNumberOfIterations = atoi( argv[7] );
    }


  optimizer->SetMaximumStepLength( steplength );
  optimizer->SetMinimumStepLength( 0.0001 );
  optimizer->SetNumberOfIterations( maxNumberOfIterations );



  optimizer->MinimizeOn();


  // Create the Command observer and register it with the optimizer.
  //
  CommandIterationUpdate::Pointer observer = CommandIterationUpdate::New();
  optimizer->AddObserver( itk::IterationEvent(), observer );



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



  OptimizerType::ParametersType finalParameters =
                    registration->GetLastTransformParameters();

  const unsigned int numberOfIterations = optimizer->GetCurrentIteration();
  const double bestValue = optimizer->GetValue();


  // Print out results
  //
  std::cout << "Result = " << std::endl;
  std::cout << " Iterations    = " << numberOfIterations << std::endl;
  std::cout << " Metric value  = " << bestValue          << std::endl;


  //  The following code is used to dump output images to files.
  //  They illustrate the final results of the registration.
  //  We will resample the moving image and write out the checkerboard image
  //  before and after registration.
  typedef itk::ResampleImageFilter<
                            MovingImageType,
                            FixedImageType >    ResampleFilterType;

  TransformType::Pointer finalTransform = TransformType::New();

  finalTransform->SetParameters( finalParameters );
  finalTransform->SetFixedParameters( transform->GetFixedParameters() );

  ResampleFilterType::Pointer resampler = ResampleFilterType::New();

  resampler->SetTransform( finalTransform );
  resampler->SetInput( movingImageReader->GetOutput() );

  FixedImageType::Pointer fixedImage = fixedImageReader->GetOutput();

  resampler->SetSize(    fixedImage->GetLargestPossibleRegion().GetSize() );
  resampler->SetOutputOrigin(  fixedImage->GetOrigin() );
  resampler->SetOutputSpacing( fixedImage->GetSpacing() );
  resampler->SetOutputDirection( fixedImage->GetDirection() );
  resampler->SetDefaultPixelValue( 100 );

  typedef  unsigned char  OutputPixelType;

  typedef itk::Image< OutputPixelType, Dimension > OutputImageType;


  typedef itk::ImageFileWriter< FixedImageType >  WriterType;


  WriterType::Pointer      writer =  WriterType::New();

  writer->SetFileName( argv[3] );

  writer->SetInput( resampler->GetOutput()   );
  writer->Update();


  typedef itk::CheckerBoardImageFilter<
    FixedImageType > CheckerBoardFilterType;

  CheckerBoardFilterType::Pointer checkerBoardFilter = CheckerBoardFilterType::New();

  CheckerBoardFilterType::PatternArrayType pattern;
  pattern[0] = 8;
  pattern[1] = 8;
  pattern[2] = 8;
  checkerBoardFilter->SetCheckerPattern( pattern );

  checkerBoardFilter->SetInput1( fixedImageReader->GetOutput() );
  checkerBoardFilter->SetInput2( resampler->GetOutput() );

  writer->SetInput( checkerBoardFilter->GetOutput() );
  resampler->SetDefaultPixelValue( 1 );

  // Compute the checkerBoardFilter image between the
  // fixed and resampled moving image.
  if( argc > 5 )
    {
    writer->SetFileName( argv[5] );
    writer->Update();
    }

  // Compute the checkerBoardFilter image between the fixed and moving image
  // before registration. Using the Transform computed by the transform
  // initializer.
  if( argc > 4 )
    {
    resampler->SetTransform( initialTransform );
    writer->SetFileName( argv[4] );
    writer->Update();
    }

  return EXIT_SUCCESS;
}
