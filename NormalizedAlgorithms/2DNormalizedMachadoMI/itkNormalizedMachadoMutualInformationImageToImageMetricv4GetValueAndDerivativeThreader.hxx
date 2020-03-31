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
#ifndef itkNormalizedMachadoMutualInformationImageToImageMetricv4GetValueAndDerivativeThreader_hxx
#define itkNormalizedMachadoMutualInformationImageToImageMetricv4GetValueAndDerivativeThreader_hxx

#include "itkNormalizedMachadoMutualInformationImageToImageMetricv4GetValueAndDerivativeThreader.h"

namespace itk
{

template <typename TDomainPartitioner, typename TImageToImageMetric, typename TNormalizedMachadoMutualInformationMetric>
void
NormalizedMachadoMutualInformationImageToImageMetricv4GetValueAndDerivativeThreader<
  TDomainPartitioner,
  TImageToImageMetric,
  TNormalizedMachadoMutualInformationMetric>::BeforeThreadedExecution()
{
  /* Most of this code needs to be here because we need to know the number
   * of threads the threader will use, which isn't known for sure until this
   * method is called. */

  /* Allocates and inits per-thread members.
   * We need a couple of these and the rest will be ignored. */
  Superclass::BeforeThreadedExecution();

  /* Store the casted pointer to avoid dynamic casting in tight loops. */
  this->m_NormalizedMachadoAssociate = dynamic_cast<TNormalizedMachadoMutualInformationMetric *>(this->m_Associate);
  if (this->m_NormalizedMachadoAssociate == nullptr)
  {
    itkExceptionMacro("Dynamic casting of associate pointer failed.");
  }

  /* Porting: these next blocks of code are from NormalizedMachadoMutualImageToImageMetric::Initialize */

  /*
   * Allocate memory for the marginal PDF and initialize values
   * to zero. The marginal PDFs are stored as std::vector.
   */
  if (this->m_NormalizedMachadoAssociate->m_MovingImageMarginalPDF.size() != this->m_NormalizedMachadoAssociate->m_NumberOfHistogramBins)
  {
    this->m_NormalizedMachadoAssociate->m_MovingImageMarginalPDF.resize(this->m_NormalizedMachadoAssociate->m_NumberOfHistogramBins, 0.0F);
  }
  else
  {
    std::fill(this->m_NormalizedMachadoAssociate->m_MovingImageMarginalPDF.begin(),
              this->m_NormalizedMachadoAssociate->m_MovingImageMarginalPDF.end(),
              PDFValueType{});
  }

  const ThreadIdType NormalizedMachadoAssociateNumThreadsUsed = this->m_NormalizedMachadoAssociate->GetNumberOfWorkUnitsUsed();
  const bool         reinitializeThreaderFixedImageMarginalPDF =
    (this->m_NormalizedMachadoAssociate->m_ThreaderFixedImageMarginalPDF.size() != NormalizedMachadoAssociateNumThreadsUsed);

  if (reinitializeThreaderFixedImageMarginalPDF)
  {
    this->m_NormalizedMachadoAssociate->m_ThreaderFixedImageMarginalPDF.resize(
      NormalizedMachadoAssociateNumThreadsUsed, std::vector<PDFValueType>(this->m_NormalizedMachadoAssociate->m_NumberOfHistogramBins, 0.0F));
  }
  else
  {
    for (ThreadIdType threadId = 0; threadId < NormalizedMachadoAssociateNumThreadsUsed; ++threadId)
    {
      std::fill(this->m_NormalizedMachadoAssociate->m_ThreaderFixedImageMarginalPDF[threadId].begin(),
                this->m_NormalizedMachadoAssociate->m_ThreaderFixedImageMarginalPDF[threadId].end(),
                PDFValueType{});
    }
  }

  const ThreadIdType localNumberOfWorkUnitsUsed = this->GetNumberOfWorkUnitsUsed();

  this->m_NormalizedMachadoAssociate->m_JointPDFSum = 0;

  JointPDFRegionType jointPDFRegion;
  // For the joint PDF define a region starting from {0,0}
  // with size {m_NumberOfHistogramBins, this->m_NumberOfHistogramBins}.
  // The dimension represents fixed image bin size
  // and moving image bin size , respectively.
  JointPDFIndexType jointPDFIndex;
  jointPDFIndex.Fill(0);
  JointPDFSizeType jointPDFSize;
  jointPDFSize.Fill(this->m_NormalizedMachadoAssociate->m_NumberOfHistogramBins);

  jointPDFRegion.SetIndex(jointPDFIndex);
  jointPDFRegion.SetSize(jointPDFSize);

  /*
   * Allocate memory for the joint PDF and joint PDF derivatives accumulator caches
   * The joint PDF and joint PDF derivatives are store as itk::Image.
   *
   * Avoid allocations if already the correct size.
   * Only recreate if size differ from last time.  If size is the same,
   * there is no need to recreate the memory
   */
  if ((this->m_NormalizedMachadoAssociate->m_ThreaderJointPDF.size() == localNumberOfWorkUnitsUsed) &&
      (jointPDFRegion == this->m_NormalizedMachadoAssociate->m_ThreaderJointPDF[0]->GetBufferedRegion()))
  {
    for (ThreadIdType threadId = 0; threadId < localNumberOfWorkUnitsUsed; ++threadId)
    {
      // Still need to reset to zero for subsequent runs
      this->m_NormalizedMachadoAssociate->m_ThreaderJointPDF[threadId]->FillBuffer(0.0);
    }
  }
  else
  {
    // By setting these values, the joint histogram physical locations will
    // correspond to intensity values.
    typename JointPDFType::PointType origin;
    origin[0] = this->m_NormalizedMachadoAssociate->m_FixedImageTrueMin;
    origin[1] = this->m_NormalizedMachadoAssociate->m_MovingImageTrueMin;
    typename JointPDFType::SpacingType spacing;
    spacing[0] = this->m_NormalizedMachadoAssociate->m_FixedImageBinSize;
    spacing[1] = this->m_NormalizedMachadoAssociate->m_MovingImageBinSize;

    this->m_NormalizedMachadoAssociate->m_ThreaderJointPDF.resize(localNumberOfWorkUnitsUsed);
    for (ThreadIdType threadId = 0; threadId < localNumberOfWorkUnitsUsed; ++threadId)
    {
      this->m_NormalizedMachadoAssociate->m_ThreaderJointPDF[threadId] = JointPDFType::New();
      this->m_NormalizedMachadoAssociate->m_ThreaderJointPDF[threadId]->SetRegions(jointPDFRegion);
      this->m_NormalizedMachadoAssociate->m_ThreaderJointPDF[threadId]->SetOrigin(origin);
      this->m_NormalizedMachadoAssociate->m_ThreaderJointPDF[threadId]->SetSpacing(spacing);
      // NOTE: true = initizize to zero
      this->m_NormalizedMachadoAssociate->m_ThreaderJointPDF[threadId]->Allocate(true);
    }
  }

  //
  // Now allocate memory according to transform type
  //
  if (!this->m_NormalizedMachadoAssociate->GetComputeDerivative())
  {
    // We only need these if we're computing derivatives.
    this->m_NormalizedMachadoAssociate->m_PRatioArray.clear();
    this->m_NormalizedMachadoAssociate->m_JointPdfIndex1DArray.clear();
    this->m_NormalizedMachadoAssociate->m_LocalDerivativeByParzenBin.clear();
    this->m_NormalizedMachadoAssociate->m_JointPDFDerivatives = nullptr;
  }

  if (this->m_NormalizedMachadoAssociate->GetComputeDerivative() && this->m_NormalizedMachadoAssociate->HasLocalSupport())
  {
    this->m_NormalizedMachadoAssociate->m_PRatioArray.assign(
      this->m_NormalizedMachadoAssociate->m_NumberOfHistogramBins * this->m_NormalizedMachadoAssociate->m_NumberOfHistogramBins, 0.0);
    this->m_NormalizedMachadoAssociate->m_JointPdfIndex1DArray.assign(this->m_NormalizedMachadoAssociate->GetNumberOfParameters(), 0);
    // Don't need this with local-support
    this->m_NormalizedMachadoAssociate->m_JointPDFDerivatives = nullptr;
    // This always has four entries because the parzen window size is fixed.
    this->m_NormalizedMachadoAssociate->m_LocalDerivativeByParzenBin.resize(4);
    // The first container cannot point to the existing derivative result
    // object
    // for efficiency, because of multi-variate metric.
    for (SizeValueType n = 0; n < 4; ++n)
    {
      this->m_NormalizedMachadoAssociate->m_LocalDerivativeByParzenBin[n].SetSize(
        this->m_NormalizedMachadoAssociate->GetNumberOfParameters());
      // Initialize to zero because we accumulate, and so skipped points will
      // behave properly
      this->m_NormalizedMachadoAssociate->m_LocalDerivativeByParzenBin[n].Fill(NumericTraits<DerivativeValueType>::ZeroValue());
    }
  }
  if (this->m_NormalizedMachadoAssociate->GetComputeDerivative() && !this->m_NormalizedMachadoAssociate->HasLocalSupport())
  {
    // Don't need this with global transforms
    this->m_NormalizedMachadoAssociate->m_PRatioArray.clear();
    this->m_NormalizedMachadoAssociate->m_JointPdfIndex1DArray.clear();
    this->m_NormalizedMachadoAssociate->m_LocalDerivativeByParzenBin.clear();

    JointPDFDerivativesRegionType jointPDFDerivativesRegion;
    {
      // For the derivatives of the joint PDF define a region starting from
      // {0,0,0}
      // with size {m_NumberOfParameters,m_NumberOfHistogramBins,
      // this->m_NumberOfHistogramBins}. The dimension represents transform
      // parameters,
      // fixed image parzen window index and moving image parzen window index,
      // respectively.
      JointPDFDerivativesIndexType jointPDFDerivativesIndex;
      jointPDFDerivativesIndex.Fill(0);
      JointPDFDerivativesSizeType jointPDFDerivativesSize;
      jointPDFDerivativesSize[0] = this->GetCachedNumberOfLocalParameters();
      jointPDFDerivativesSize[1] = this->m_NormalizedMachadoAssociate->m_NumberOfHistogramBins;
      jointPDFDerivativesSize[2] = this->m_NormalizedMachadoAssociate->m_NumberOfHistogramBins;

      jointPDFDerivativesRegion.SetIndex(jointPDFDerivativesIndex);
      jointPDFDerivativesRegion.SetSize(jointPDFDerivativesSize);
    }

    // Set the regions and allocate
    if (this->m_NormalizedMachadoAssociate->m_JointPDFDerivatives.IsNull() ||
        (this->m_NormalizedMachadoAssociate->m_JointPDFDerivatives->GetBufferedRegion() != jointPDFDerivativesRegion))
    {
      this->m_NormalizedMachadoAssociate->m_JointPDFDerivatives = JointPDFDerivativesType::New();
      this->m_NormalizedMachadoAssociate->m_JointPDFDerivatives->SetRegions(jointPDFDerivativesRegion);
      this->m_NormalizedMachadoAssociate->m_JointPDFDerivatives->Allocate(true);
    }
    else
    {
      // Initialize to zero for accumulation
      this->m_NormalizedMachadoAssociate->m_JointPDFDerivatives->FillBuffer(0.0F);
    }
    if ((this->m_NormalizedMachadoAssociate->m_ThreaderDerivativeManager.size() != localNumberOfWorkUnitsUsed))
    {
      this->m_NormalizedMachadoAssociate->m_ThreaderDerivativeManager.resize(localNumberOfWorkUnitsUsed);
    }
    for (ThreadIdType threadId = 0; threadId < localNumberOfWorkUnitsUsed; ++threadId)
    {
      this->m_NormalizedMachadoAssociate->m_ThreaderDerivativeManager[threadId].Initialize(
        // A heuristic that assumues memory for 2x size of
        // m_JointPDFDerivati efficient and easy to make, so
        // split it accross all the threads.  A work unit of at least 400 is needed
        // when the thread size approaches the number of histograms so that the
        // there is enough work to be done between thread lockings.
        std::max<size_t>(500,
                         this->m_NormalizedMachadoAssociate->m_NumberOfHistogramBins *
                           this->m_NormalizedMachadoAssociate->m_NumberOfHistogramBins / localNumberOfWorkUnitsUsed),
        this->GetCachedNumberOfLocalParameters(),
        // Need address of the lock
        &this->m_NormalizedMachadoAssociate->m_JointPDFDerivativesLock,
        this->m_NormalizedMachadoAssociate->m_JointPDFDerivatives);
    }
  }
}

template <typename TDomainPartitioner, typename TImageToImageMetric, typename TNormalizedMachadoMutualInformationMetric>
bool
NormalizedMachadoMutualInformationImageToImageMetricv4GetValueAndDerivativeThreader<
  TDomainPartitioner,
  TImageToImageMetric,
  TNormalizedMachadoMutualInformationMetric>::ProcessPoint(const VirtualIndexType & virtualIndex,
                                                const VirtualPointType & virtualPoint,
                                                const FixedImagePointType &,
                                                const FixedImagePixelType & fixedImageValue,
                                                const FixedImageGradientType &,
                                                const MovingImagePointType &,
                                                const MovingImagePixelType &    movingImageValue,
                                                const MovingImageGradientType & movingImageGradient,
                                                MeasureType &,
                                                DerivativeType &,
                                                const ThreadIdType threadId) const
{
  const bool doComputeDerivative = this->m_NormalizedMachadoAssociate->GetComputeDerivative();
  /**
   * Compute this sample's contribution to the marginal
   *   and joint distributions.
   *
   */
  if (movingImageValue < this->m_NormalizedMachadoAssociate->m_MovingImageTrueMin)
  {
    return false;
  }
  else if (movingImageValue > this->m_NormalizedMachadoAssociate->m_MovingImageTrueMax)
  {
    return false;
  }

  // Determine parzen window arguments (see eqn 6 of NormalizedMachado paper [2]).
  const PDFValueType movingImageParzenWindowTerm = movingImageValue / this->m_NormalizedMachadoAssociate->m_MovingImageBinSize -
                                                   this->m_NormalizedMachadoAssociate->m_MovingImageNormalizedMin;
  auto movingImageParzenWindowIndex = static_cast<OffsetValueType>(movingImageParzenWindowTerm);

  // Make sure the extreme values are in valid bins
  if (movingImageParzenWindowIndex < 2)
  {
    movingImageParzenWindowIndex = 2;
  }
  else
  {
    const OffsetValueType nindex = static_cast<OffsetValueType>(this->m_NormalizedMachadoAssociate->m_NumberOfHistogramBins) - 3;
    if (movingImageParzenWindowIndex > nindex)
    {
      movingImageParzenWindowIndex = nindex;
    }
  }
  // Move the pointer to the first affected bin
  OffsetValueType       pdfMovingIndex = static_cast<OffsetValueType>(movingImageParzenWindowIndex) - 1;
  const OffsetValueType pdfMovingIndexMax = static_cast<OffsetValueType>(movingImageParzenWindowIndex) + 2;

  const OffsetValueType fixedImageParzenWindowIndex =
    this->m_NormalizedMachadoAssociate->ComputeSingleFixedImageParzenWindowIndex(fixedImageValue);

  // Since a zero-order BSpline (box car) kernel is used for
  // the fixed image marginal pdf, we need only increment the
  // fixedImageParzenWindowIndex by value of 1.0.
  this->m_NormalizedMachadoAssociate->m_ThreaderFixedImageMarginalPDF[threadId][fixedImageParzenWindowIndex] += 1;

  /**
   * The region of support of the parzen window determines which bins
   * of the joint PDF are effected by the pair of image values.
   * Since we are using a cubic spline for the moving image parzen
   * window, four bins are effected.  The fixed image parzen window is
   * a zero-order spline (box car) and thus effects only one bin.
   *
   *  The PDF is arranged so that moving image bins corresponds to the
   * zero-th (column) dimension and the fixed image bins corresponds
   * to the first (row) dimension.
   */
  PDFValueType movingImageParzenWindowArg =
    static_cast<PDFValueType>(pdfMovingIndex) - static_cast<PDFValueType>(movingImageParzenWindowTerm);

  // Pointer to affected bin to be updated
  JointPDFValueType * pdfPtr = this->m_NormalizedMachadoAssociate->m_ThreaderJointPDF[threadId]->GetBufferPointer() +
                               (fixedImageParzenWindowIndex * this->m_NormalizedMachadoAssociate->m_NumberOfHistogramBins) +
                               pdfMovingIndex;

  OffsetValueType localDerivativeOffset = 0;
  // Store the pdf indices for this point.
  // Just store the starting pdfMovingIndex and we'll iterate later
  // over the next four to collect results.
  if (doComputeDerivative && (this->m_NormalizedMachadoAssociate->HasLocalSupport()))
  {
    const OffsetValueType jointPdfIndex1D =
      pdfMovingIndex + (fixedImageParzenWindowIndex * this->m_NormalizedMachadoAssociate->m_NumberOfHistogramBins);
    localDerivativeOffset = this->m_NormalizedMachadoAssociate->ComputeParameterOffsetFromVirtualIndex(
      virtualIndex, this->GetCachedNumberOfLocalParameters());
    for (NumberOfParametersType i = 0, numLocalParameters = this->GetCachedNumberOfLocalParameters();
         i < numLocalParameters;
         ++i)
    {
      this->m_NormalizedMachadoAssociate->m_JointPdfIndex1DArray[localDerivativeOffset + i] = jointPdfIndex1D;
    }
  }

  // Compute the transform Jacobian.
  using JacobianReferenceType = JacobianType &;
  JacobianReferenceType jacobian = this->m_GetValueAndDerivativePerThreadVariables[threadId].MovingTransformJacobian;
  if (doComputeDerivative)
  {
    JacobianReferenceType jacobianPositional =
      this->m_GetValueAndDerivativePerThreadVariables[threadId].MovingTransformJacobianPositional;
    this->m_NormalizedMachadoAssociate->GetMovingTransform()->ComputeJacobianWithRespectToParametersCachedTemporaries(
      virtualPoint, jacobian, jacobianPositional);
  }

  SizeValueType movingParzenBin = 0;

  const bool transformIsDisplacement = this->m_NormalizedMachadoAssociate->m_MovingTransform->GetTransformCategory() ==
                                       MovingTransformType::TransformCategoryType::DisplacementField;
  while (pdfMovingIndex <= pdfMovingIndexMax)
  {
    const auto val =
      static_cast<PDFValueType>(this->m_NormalizedMachadoAssociate->m_CubicBSplineKernel->Evaluate(movingImageParzenWindowArg));
    *(pdfPtr++) += val;

    if (doComputeDerivative)
    {
      // Compute the cubicBSplineDerivative for later repeated use.
      const PDFValueType cubicBSplineDerivativeValue =
        this->m_NormalizedMachadoAssociate->m_CubicBSplineDerivativeKernel->Evaluate(movingImageParzenWindowArg);


      if (transformIsDisplacement)
      {
        // Pointer to local derivative partial result container.
        // Not used with global support transforms.
        // ptr to where the derivative result should go, for efficiency
        DerivativeValueType * localSupportDerivativeResultPtr =
          &(this->m_NormalizedMachadoAssociate->m_LocalDerivativeByParzenBin[movingParzenBin][localDerivativeOffset]);
        // Compute PDF derivative contribution.

        this->ComputePDFDerivativesLocalSupportTransform(
          jacobian, movingImageGradient, cubicBSplineDerivativeValue, localSupportDerivativeResultPtr);
      }
      else
      {
        // Update bins in the PDF derivatives for the current intensity pair
        const OffsetValueType ThisIndexOffset =
          (fixedImageParzenWindowIndex * this->m_NormalizedMachadoAssociate->m_JointPDFDerivatives->GetOffsetTable()[2]) +
          (pdfMovingIndex * this->m_NormalizedMachadoAssociate->m_JointPDFDerivatives->GetOffsetTable()[1]);

        PDFValueType * derivativeContributionPtr =
          this->m_NormalizedMachadoAssociate->m_ThreaderDerivativeManager[threadId].GetNextElementAndAddOffset(ThisIndexOffset);
        for (NumberOfParametersType mu = 0, maxElement = this->GetCachedNumberOfLocalParameters(); mu < maxElement;
             ++mu)
        {
          PDFValueType innerProduct = 0.0;
          for (SizeValueType dim = 0, lastDim = this->m_NormalizedMachadoAssociate->MovingImageDimension; dim < lastDim; ++dim)
          {
            innerProduct += jacobian[dim][mu] * movingImageGradient[dim];
          }

          *(derivativeContributionPtr) = innerProduct * cubicBSplineDerivativeValue;
          ++derivativeContributionPtr;
        }
        this->m_NormalizedMachadoAssociate->m_ThreaderDerivativeManager[threadId].CheckAndReduceIfNecessary();
      }
    }

    movingImageParzenWindowArg += 1.0;
    ++pdfMovingIndex;
    ++movingParzenBin;
  }

  // have to do this here since we're returning false
  this->m_GetValueAndDerivativePerThreadVariables[threadId].NumberOfValidPoints++;

  // Return false to avoid the storage of results in parent class.
  return false;
}

template <typename TDomainPartitioner, typename TImageToImageMetric, typename TNormalizedMachadoMutualInformationMetric>
void
NormalizedMachadoMutualInformationImageToImageMetricv4GetValueAndDerivativeThreader<TDomainPartitioner,
                                                                         TImageToImageMetric,
                                                                         TNormalizedMachadoMutualInformationMetric>::
  ComputePDFDerivativesLocalSupportTransform(const JacobianType &            jacobian,
                                             const MovingImageGradientType & movingImageGradient,
                                             const PDFValueType &            cubicBSplineDerivativeValue,
                                             DerivativeValueType *           localSupportDerivativeResultPtr) const
{
  for (NumberOfParametersType mu = 0, maxElement = this->GetCachedNumberOfLocalParameters(); mu < maxElement; ++mu)
  {
    PDFValueType innerProduct = 0.0;
    for (SizeValueType dim = 0, lastDim = this->m_NormalizedMachadoAssociate->MovingImageDimension; dim < lastDim; ++dim)
    {
      innerProduct += jacobian[dim][mu] * movingImageGradient[dim];
    }

    const PDFValueType derivativeContribution = innerProduct * cubicBSplineDerivativeValue;
    *(localSupportDerivativeResultPtr) += derivativeContribution;
    localSupportDerivativeResultPtr++;
  }
}

template <typename TDomainPartitioner, typename TImageToImageMetric, typename TNormalizedMachadoMutualInformationMetric>
void
NormalizedMachadoMutualInformationImageToImageMetricv4GetValueAndDerivativeThreader<
  TDomainPartitioner,
  TImageToImageMetric,
  TNormalizedMachadoMutualInformationMetric>::AfterThreadedExecution()
{
  const ThreadIdType localNumberOfWorkUnitsUsed = this->GetNumberOfWorkUnitsUsed();
  /* Store the number of valid points in the enclosing class
   * m_NumberOfValidPoints by collecting the valid points per thread.
   * We do this here because we're skipping Superclass::AfterThreadedExecution*/
  this->m_NormalizedMachadoAssociate->m_NumberOfValidPoints = NumericTraits<SizeValueType>::ZeroValue();
  for (ThreadIdType threadId = 0; threadId < localNumberOfWorkUnitsUsed; ++threadId)
  {
    this->m_NormalizedMachadoAssociate->m_NumberOfValidPoints +=
      this->m_GetValueAndDerivativePerThreadVariables[threadId].NumberOfValidPoints;
  }

  /* Porting: This code is from
   * NormalizedMachadoMutualInformationImageToImageMetric::GetValueAndDerivativeThreadPostProcess */
  /* Post-processing that is common the GetValue and GetValueAndDerivative */
  this->m_NormalizedMachadoAssociate->GetValueCommonAfterThreadedExecution();

  if (this->m_NormalizedMachadoAssociate->GetComputeDerivative() && (!this->m_NormalizedMachadoAssociate->HasLocalSupport()))
  {
    // This entire block of code is used to accumulate the per-thread buffers
    // into 1 thread.
    // For this thread, how many histogram elements are there?
    const NumberOfParametersType rowSize =
      this->GetCachedNumberOfLocalParameters() * this->m_NormalizedMachadoAssociate->m_NumberOfHistogramBins;
    const SizeValueType histogramTotalElementsSize = rowSize * this->m_NormalizedMachadoAssociate->m_NumberOfHistogramBins;

    // NOTE:  Negative 1 so that accumulators can all be positive accumulators
    const PDFValueType nFactor =
      -1.0 / (this->m_NormalizedMachadoAssociate->m_MovingImageBinSize * this->m_NormalizedMachadoAssociate->GetNumberOfValidPoints());

    JointPDFDerivativesValueType * const accumulatorPdfDPtrStart =
      this->m_NormalizedMachadoAssociate->m_JointPDFDerivatives->GetBufferPointer();
    JointPDFDerivativesValueType *             accumulatorPdfDPtr = accumulatorPdfDPtrStart;
    JointPDFDerivativesValueType const * const tempThreadPdfDPtrEnd =
      accumulatorPdfDPtrStart + histogramTotalElementsSize;
    while (accumulatorPdfDPtr < tempThreadPdfDPtrEnd)
    {
      *(accumulatorPdfDPtr++) *= nFactor;
    }
  }

  // Collect and compute results.
  // Value and derivative are stored in member vars.
  this->m_NormalizedMachadoAssociate->ComputeResults();
}

} // end namespace itk

#endif